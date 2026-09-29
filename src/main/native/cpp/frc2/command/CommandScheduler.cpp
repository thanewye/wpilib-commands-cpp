// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Modified to match the Java CommandScheduler's iteration order and run-loop
// deferral of schedule and cancel requests.

#include "frc2/command/CommandScheduler.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <frc/RobotBase.h>
#include <frc/RobotState.h>
#include <frc/TimedRobot.h>
#include <frc/livewindow/LiveWindow.h>
#include <hal/FRCUsageReporting.h>
#include <hal/HALBase.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/StringArrayTopic.h>
#include <wpi/DenseMap.h>
#include <wpi/MapVector.h>
#include <wpi/SmallVector.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include "frc2/command/CommandPtr.h"
#include "frc2/command/Subsystem.h"

using namespace frc2;

class CommandScheduler::Impl {
 public:
  // A set of the currently-running commands.
  wpi::SmallVector<Command*, 12> scheduledCommands;

  // A map from required subsystems to their requiring commands.  Also used as a
  // set of the currently-required subsystems.
  wpi::MapVector<Subsystem*, Command*> requirements;

  // A map from subsystems registered with the scheduler to their default
  // commands.  Also used as a list of currently-registered subsystems.
  wpi::MapVector<Subsystem*, std::unique_ptr<Command>> subsystems;

  frc::EventLoop defaultButtonLoop;
  // The set of currently-registered buttons that will be polled every
  // iteration.
  frc::EventLoop* activeButtonLoop{&defaultButtonLoop};

  bool disabled{false};

  // Lists of user-supplied actions to be executed on scheduling events for
  // every command.
  wpi::SmallVector<Action, 4> initActions;
  wpi::SmallVector<Action, 4> executeActions;
  wpi::SmallVector<InterruptAction, 4> interruptActions;
  wpi::SmallVector<Action, 4> finishActions;

  // Map of Command* -> CommandPtr for CommandPtrs transferred to the scheduler
  // via Schedule(CommandPtr&&). These are erased (destroyed) at the very end of
  // the loop cycle when the command lifecycle is complete.
  wpi::DenseMap<Command*, CommandPtr> ownedCommands;

  bool inRunLoop{false};
  wpi::SmallVector<Command*, 4> toSchedule;
  wpi::SmallVector<Command*, 4> toCancelCommands;
  wpi::SmallVector<std::optional<Command*>, 4> toCancelInterruptors;
  wpi::SmallVector<Command*, 4> endingCommands;

  bool IsCommandScheduled(const Command* command) const {
    return std::find(scheduledCommands.begin(), scheduledCommands.end(),
                     command) != scheduledCommands.end();
  }

  void EraseScheduledCommand(const Command* command) {
    auto scheduledCommand = std::find(scheduledCommands.begin(),
                                      scheduledCommands.end(), command);
    if (scheduledCommand != scheduledCommands.end()) {
      scheduledCommands.erase(scheduledCommand);
    }
  }

  bool IsEnding(const Command* command) const {
    return std::find(endingCommands.begin(), endingCommands.end(), command) !=
           endingCommands.end();
  }

  void RemoveEnding(const Command* command) {
    auto endingCommand =
        std::find(endingCommands.begin(), endingCommands.end(), command);
    if (endingCommand != endingCommands.end()) {
      endingCommands.erase(endingCommand);
    }
  }

  void ReleaseRequirementsOf(const Command* command) {
    for (auto&& requirement : command->GetRequirements()) {
      requirements.erase(requirement);
    }
  }
};

template <typename TMap, typename TKey>
static bool ContainsKey(const TMap& map, TKey keyToCheck) {
  return map.find(keyToCheck) != map.end();
}

CommandScheduler::CommandScheduler()
    : m_impl(new Impl), m_watchdog(frc::TimedRobot::kDefaultPeriod, [] {
        std::puts("CommandScheduler loop time overrun.");
      }) {
  HAL_Report(HALUsageReporting::kResourceType_Command,
             HALUsageReporting::kCommand2_Scheduler);
  wpi::SendableRegistry::AddLW(this, "Scheduler");
  frc::LiveWindow::SetEnabledCallback([this] {
    this->Disable();
    this->CancelAll();
  });
  frc::LiveWindow::SetDisabledCallback([this] { this->Enable(); });
}

CommandScheduler::~CommandScheduler() {
  wpi::SendableRegistry::Remove(this);
  frc::LiveWindow::SetEnabledCallback(nullptr);
  frc::LiveWindow::SetDisabledCallback(nullptr);

  std::unique_ptr<Impl>().swap(m_impl);
}

CommandScheduler& CommandScheduler::GetInstance() {
  static CommandScheduler scheduler;
  return scheduler;
}

void CommandScheduler::SetPeriod(units::second_t period) {
  m_watchdog.SetTimeout(period);
}

frc::EventLoop* CommandScheduler::GetActiveButtonLoop() const {
  return m_impl->activeButtonLoop;
}

void CommandScheduler::SetActiveButtonLoop(frc::EventLoop* loop) {
  m_impl->activeButtonLoop = loop;
}

frc::EventLoop* CommandScheduler::GetDefaultButtonLoop() const {
  return &(m_impl->defaultButtonLoop);
}

void CommandScheduler::Schedule(Command* command) {
  if (m_impl->inRunLoop) {
    if (std::find(m_impl->toSchedule.begin(), m_impl->toSchedule.end(),
                  command) == m_impl->toSchedule.end()) {
      m_impl->toSchedule.emplace_back(command);
    }
    return;
  }

  RequireUngrouped(command);

  if (m_impl->disabled || m_impl->IsCommandScheduled(command) ||
      (frc::RobotState::IsDisabled() && !command->RunsWhenDisabled())) {
    return;
  }

  const auto requirements = command->GetRequirements();

  bool isDisjoint = true;
  for (auto&& requirement : requirements) {
    if (ContainsKey(m_impl->requirements, requirement)) {
      isDisjoint = false;
      break;
    }
  }

  if (!isDisjoint) {
    for (auto&& requirement : requirements) {
      Command* requiring = Requiring(requirement);
      if (requiring != nullptr &&
          requiring->GetInterruptionBehavior() ==
              Command::InterruptionBehavior::kCancelIncoming) {
        return;
      }
    }
    for (auto&& requirement : requirements) {
      Command* requiring = Requiring(requirement);
      if (requiring != nullptr) {
        Cancel(requiring, std::make_optional(command));
      }
    }
  }

  m_impl->scheduledCommands.emplace_back(command);
  for (auto&& requirement : requirements) {
    m_impl->requirements[requirement] = command;
  }
  command->Initialize();
  for (auto&& action : m_impl->initActions) {
    action(*command);
  }
  m_watchdog.AddEpoch(command->GetName() + ".Initialize()");
}

void CommandScheduler::Schedule(std::span<Command* const> commands) {
  for (auto command : commands) {
    Schedule(command);
  }
}

void CommandScheduler::Schedule(std::initializer_list<Command*> commands) {
  for (auto command : commands) {
    Schedule(command);
  }
}

void CommandScheduler::Schedule(const CommandPtr& command) {
  Schedule(command.get());
}

void CommandScheduler::Schedule(CommandPtr&& command) {
  auto ptr = command.get();
  m_impl->ownedCommands.try_emplace(ptr, std::move(command));
  Schedule(ptr);
}

void CommandScheduler::Run() {
  if (m_impl->disabled) {
    return;
  }

  m_watchdog.Reset();

  // Run the periodic method of all registered subsystems.
  for (auto&& subsystem : m_impl->subsystems) {
    subsystem.first->Periodic();
    if constexpr (frc::RobotBase::IsSimulation()) {
      subsystem.first->SimulationPeriodic();
    }
    m_watchdog.AddEpoch(subsystem.first->GetName() + ".Periodic()");
  }

  // Cache the active instance to avoid concurrency problems if SetActiveLoop()
  // is called from inside the button bindings.
  frc::EventLoop* loopCache = m_impl->activeButtonLoop;
  // Poll buttons for new commands to add.
  loopCache->Poll();
  m_watchdog.AddEpoch("buttons.Run()");

  m_impl->inRunLoop = true;
  bool isDisabled = frc::RobotState::IsDisabled();
  auto& scheduledCommands = m_impl->scheduledCommands;
  for (size_t commandIndex = 0; commandIndex < scheduledCommands.size();) {
    Command* command = scheduledCommands[commandIndex];

    if (isDisabled && !command->RunsWhenDisabled()) {
      Cancel(command, std::nullopt);
      ++commandIndex;
      continue;
    }

    command->Execute();
    for (auto&& action : m_impl->executeActions) {
      action(*command);
    }
    m_watchdog.AddEpoch(command->GetName() + ".Execute()");

    if (command->IsFinished()) {
      m_impl->endingCommands.emplace_back(command);
      command->End(false);
      for (auto&& action : m_impl->finishActions) {
        action(*command);
      }
      m_impl->RemoveEnding(command);
      scheduledCommands.erase(scheduledCommands.begin() + commandIndex);
      m_impl->ReleaseRequirementsOf(command);

      m_watchdog.AddEpoch(command->GetName() + ".End(false)");
      // remove owned commands after everything else is done
      m_impl->ownedCommands.erase(command);
    } else {
      ++commandIndex;
    }
  }
  m_impl->inRunLoop = false;

  for (size_t scheduleIndex = 0; scheduleIndex < m_impl->toSchedule.size();
       ++scheduleIndex) {
    Schedule(m_impl->toSchedule[scheduleIndex]);
  }
  for (size_t cancelIndex = 0; cancelIndex < m_impl->toCancelCommands.size();
       ++cancelIndex) {
    Cancel(m_impl->toCancelCommands[cancelIndex],
           m_impl->toCancelInterruptors[cancelIndex]);
  }
  m_impl->toSchedule.clear();
  m_impl->toCancelCommands.clear();
  m_impl->toCancelInterruptors.clear();

  // Add default commands for un-required registered subsystems.
  for (auto&& subsystem : m_impl->subsystems) {
    if (!ContainsKey(m_impl->requirements, subsystem.first) &&
        subsystem.second) {
      Schedule({subsystem.second.get()});
    }
  }

  m_watchdog.Disable();
  if (m_watchdog.IsExpired()) {
    m_watchdog.PrintEpochs();
  }
}

void CommandScheduler::RegisterSubsystem(Subsystem* subsystem) {
  if (m_impl->subsystems.find(subsystem) != m_impl->subsystems.end()) {
    std::puts("Tried to register an already-registered subsystem");
    return;
  }

  m_impl->subsystems[subsystem] = nullptr;
}

void CommandScheduler::UnregisterSubsystem(Subsystem* subsystem) {
  m_impl->subsystems.erase(subsystem);
}

void CommandScheduler::RegisterSubsystem(
    std::initializer_list<Subsystem*> subsystems) {
  for (auto* subsystem : subsystems) {
    RegisterSubsystem(subsystem);
  }
}

void CommandScheduler::RegisterSubsystem(
    std::span<Subsystem* const> subsystems) {
  for (auto* subsystem : subsystems) {
    RegisterSubsystem(subsystem);
  }
}

void CommandScheduler::UnregisterSubsystem(
    std::initializer_list<Subsystem*> subsystems) {
  for (auto* subsystem : subsystems) {
    UnregisterSubsystem(subsystem);
  }
}

void CommandScheduler::UnregisterSubsystem(
    std::span<Subsystem* const> subsystems) {
  for (auto* subsystem : subsystems) {
    UnregisterSubsystem(subsystem);
  }
}

void CommandScheduler::UnregisterAllSubsystems() {
  m_impl->subsystems.clear();
}

void CommandScheduler::SetDefaultCommand(Subsystem* subsystem,
                                         CommandPtr&& defaultCommand) {
  if (!defaultCommand.get()->HasRequirement(subsystem)) {
    throw FRC_MakeError(frc::err::CommandIllegalUse, "{}",
                        "Default commands must require their subsystem!");
  }
  RequireUngrouped(defaultCommand.get());

  SetDefaultCommandImpl(subsystem, std::move(defaultCommand).Unwrap());
}

void CommandScheduler::RemoveDefaultCommand(Subsystem* subsystem) {
  m_impl->subsystems[subsystem] = nullptr;
}

Command* CommandScheduler::GetDefaultCommand(const Subsystem* subsystem) const {
  auto&& find = m_impl->subsystems.find(const_cast<Subsystem*>(subsystem));
  if (find != m_impl->subsystems.end()) {
    return find->second.get();
  } else {
    return nullptr;
  }
}

void CommandScheduler::Cancel(Command* command,
                              std::optional<Command*> interruptor) {
  if (!m_impl) {
    return;
  }
  if (m_impl->IsEnding(command)) {
    return;
  }
  if (m_impl->inRunLoop) {
    m_impl->toCancelCommands.emplace_back(command);
    m_impl->toCancelInterruptors.emplace_back(interruptor);
    return;
  }
  if (!IsScheduled(command)) {
    return;
  }
  m_impl->endingCommands.emplace_back(command);
  command->End(true);
  for (auto&& action : m_impl->interruptActions) {
    action(*command, interruptor);
  }
  m_impl->RemoveEnding(command);
  m_impl->EraseScheduledCommand(command);
  m_impl->ReleaseRequirementsOf(command);
  m_watchdog.AddEpoch(command->GetName() + ".End(true)");
}

void CommandScheduler::Cancel(Command* command) {
  Cancel(command, std::nullopt);
}

void CommandScheduler::Cancel(const CommandPtr& command) {
  Cancel(command.get());
}

void CommandScheduler::Cancel(std::span<Command* const> commands) {
  for (auto command : commands) {
    Cancel(command);
  }
}

void CommandScheduler::Cancel(std::initializer_list<Command*> commands) {
  for (auto command : commands) {
    Cancel(command);
  }
}

void CommandScheduler::CancelAll() {
  wpi::SmallVector<Command*, 16> commands;
  for (auto&& command : m_impl->scheduledCommands) {
    commands.emplace_back(command);
  }
  Cancel(commands);
}

bool CommandScheduler::IsScheduled(
    std::span<const Command* const> commands) const {
  for (auto command : commands) {
    if (!IsScheduled(command)) {
      return false;
    }
  }
  return true;
}

bool CommandScheduler::IsScheduled(
    std::initializer_list<const Command*> commands) const {
  for (auto command : commands) {
    if (!IsScheduled(command)) {
      return false;
    }
  }
  return true;
}

bool CommandScheduler::IsScheduled(const Command* command) const {
  return m_impl->IsCommandScheduled(command);
}

bool CommandScheduler::IsScheduled(const CommandPtr& command) const {
  return m_impl->IsCommandScheduled(command.get());
}

Command* CommandScheduler::Requiring(const Subsystem* subsystem) const {
  auto find = m_impl->requirements.find(const_cast<Subsystem*>(subsystem));
  if (find != m_impl->requirements.end()) {
    return find->second;
  } else {
    return nullptr;
  }
}

void CommandScheduler::Disable() {
  m_impl->disabled = true;
}

void CommandScheduler::Enable() {
  m_impl->disabled = false;
}

void CommandScheduler::PrintWatchdogEpochs() {
  m_watchdog.PrintEpochs();
}

void CommandScheduler::OnCommandInitialize(Action action) {
  m_impl->initActions.emplace_back(std::move(action));
}

void CommandScheduler::OnCommandExecute(Action action) {
  m_impl->executeActions.emplace_back(std::move(action));
}

void CommandScheduler::OnCommandInterrupt(Action action) {
  m_impl->interruptActions.emplace_back(
      [action = std::move(action)](const Command& command,
                                   const std::optional<Command*>& interruptor) {
        action(command);
      });
}

void CommandScheduler::OnCommandInterrupt(InterruptAction action) {
  m_impl->interruptActions.emplace_back(std::move(action));
}

void CommandScheduler::OnCommandFinish(Action action) {
  m_impl->finishActions.emplace_back(std::move(action));
}

void CommandScheduler::RequireUngrouped(const Command* command) {
  auto stacktrace = command->GetPreviousCompositionSite();
  if (stacktrace.has_value()) {
    throw FRC_MakeError(frc::err::CommandIllegalUse,
                        "Commands that have been composed may not be added to "
                        "another composition or scheduled individually!"
                        "\nOriginally composed at:\n{}",
                        stacktrace.value());
  }
}

void CommandScheduler::RequireUngrouped(
    std::span<const std::unique_ptr<Command>> commands) {
  for (auto&& command : commands) {
    RequireUngrouped(command.get());
  }
}

void CommandScheduler::RequireUngrouped(
    std::initializer_list<const Command*> commands) {
  for (auto&& command : commands) {
    RequireUngrouped(command);
  }
}

void CommandScheduler::RequireUngroupedAndUnscheduled(const Command* command) {
  if (IsScheduled(command)) {
    throw FRC_MakeError(frc::err::CommandIllegalUse,
                        "Commands that have been scheduled individually may "
                        "not be added to another composition!");
  }
  RequireUngrouped(command);
}

void CommandScheduler::RequireUngroupedAndUnscheduled(
    std::span<const std::unique_ptr<Command>> commands) {
  for (auto&& command : commands) {
    RequireUngroupedAndUnscheduled(command.get());
  }
}

void CommandScheduler::RequireUngroupedAndUnscheduled(
    std::initializer_list<const Command*> commands) {
  for (auto&& command : commands) {
    RequireUngroupedAndUnscheduled(command);
  }
}

void CommandScheduler::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Scheduler");
  builder.AddStringArrayProperty(
      "Names",
      [this]() mutable {
        std::vector<std::string> names;
        for (Command* command : m_impl->scheduledCommands) {
          names.emplace_back(command->GetName());
        }
        return names;
      },
      nullptr);
  builder.AddIntegerArrayProperty(
      "Ids",
      [this]() mutable {
        std::vector<int64_t> ids;
        for (Command* command : m_impl->scheduledCommands) {
          uintptr_t ptrTmp = reinterpret_cast<uintptr_t>(command);
          ids.emplace_back(static_cast<int64_t>(ptrTmp));
        }
        return ids;
      },
      nullptr);
  builder.AddIntegerArrayProperty(
      "Cancel", []() { return std::vector<int64_t>{}; },
      [this](std::span<const int64_t> toCancel) mutable {
        for (auto cancel : toCancel) {
          uintptr_t ptrTmp = static_cast<uintptr_t>(cancel);
          Command* command = reinterpret_cast<Command*>(ptrTmp);
          if (m_impl->IsCommandScheduled(command)) {
            Cancel(command);
          }
        }
      });
}

void CommandScheduler::SetDefaultCommandImpl(Subsystem* subsystem,
                                             std::unique_ptr<Command> command) {
  if (command->GetInterruptionBehavior() ==
      Command::InterruptionBehavior::kCancelIncoming) {
    std::puts(
        "Registering a non-interruptible default command!\n"
        "This will likely prevent any other commands from "
        "requiring this subsystem.");
    // Warn, but allow -- there might be a use case for this.
  }
  m_impl->subsystems[subsystem] = std::move(command);
}
