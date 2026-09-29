// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "CommandTestBase.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc2/command/RunCommand.h"

using namespace frc2;

class JavaSchedulerParityTest : public CommandTestBase {};

namespace {
constexpr int kMoreThanSmallSetInlineCapacity = 20;

template <typename T>
std::vector<T*> InDescendingAddressOrder(
    const std::vector<std::unique_ptr<T>>& owners) {
  std::vector<T*> pointers;
  for (auto&& owner : owners) {
    pointers.emplace_back(owner.get());
  }
  std::sort(pointers.begin(), pointers.end(), std::greater<T*>());
  return pointers;
}
}  // namespace

TEST_F(JavaSchedulerParityTest, PeriodicRunsInRegistrationOrder) {
  CommandScheduler scheduler = GetScheduler();
  std::vector<Subsystem*> periodicOrder;
  std::vector<std::unique_ptr<TestSubsystem>> subsystemOwners;
  for (int subsystemId = 0; subsystemId < kMoreThanSmallSetInlineCapacity;
       ++subsystemId) {
    auto periodicSubsystemSlot = std::make_shared<Subsystem*>(nullptr);
    subsystemOwners.emplace_back(
        std::make_unique<TestSubsystem>([&periodicOrder, periodicSubsystemSlot] {
          periodicOrder.emplace_back(*periodicSubsystemSlot);
        }));
    *periodicSubsystemSlot = subsystemOwners.back().get();
  }
  std::vector<TestSubsystem*> registrationOrder =
      InDescendingAddressOrder(subsystemOwners);
  for (auto* subsystem : registrationOrder) {
    scheduler.RegisterSubsystem(subsystem);
  }

  scheduler.Run();

  ASSERT_EQ(periodicOrder.size(), registrationOrder.size());
  EXPECT_TRUE(std::equal(periodicOrder.begin(), periodicOrder.end(),
                         registrationOrder.begin()));
}

TEST_F(JavaSchedulerParityTest, ExecuteRunsInSchedulingOrder) {
  CommandScheduler scheduler = GetScheduler();
  std::vector<Command*> executeOrder;
  std::vector<std::unique_ptr<FunctionalCommand>> commandOwners;
  for (int commandId = 0; commandId < kMoreThanSmallSetInlineCapacity;
       ++commandId) {
    auto executeCommandSlot = std::make_shared<Command*>(nullptr);
    commandOwners.emplace_back(std::make_unique<FunctionalCommand>(
        [] {},
        [&executeOrder, executeCommandSlot] {
          executeOrder.emplace_back(*executeCommandSlot);
        },
        [](bool) {}, [] { return false; }));
    *executeCommandSlot = commandOwners.back().get();
  }
  std::vector<FunctionalCommand*> schedulingOrder =
      InDescendingAddressOrder(commandOwners);
  for (auto* command : schedulingOrder) {
    scheduler.Schedule(command);
  }

  scheduler.Run();

  ASSERT_EQ(executeOrder.size(), schedulingOrder.size());
  EXPECT_TRUE(std::equal(executeOrder.begin(), executeOrder.end(),
                         schedulingOrder.begin()));
  scheduler.CancelAll();
}

TEST_F(JavaSchedulerParityTest, DefaultCommandsScheduleInRegistrationOrder) {
  CommandScheduler scheduler = GetScheduler();
  std::vector<Subsystem*> initializeOrder;
  std::vector<std::unique_ptr<TestSubsystem>> subsystemOwners;
  for (int subsystemId = 0; subsystemId < kMoreThanSmallSetInlineCapacity;
       ++subsystemId) {
    subsystemOwners.emplace_back(std::make_unique<TestSubsystem>());
  }
  std::vector<TestSubsystem*> registrationOrder =
      InDescendingAddressOrder(subsystemOwners);
  for (auto* subsystem : registrationOrder) {
    scheduler.RegisterSubsystem(subsystem);
  }
  for (auto* subsystem : registrationOrder) {
    scheduler.SetDefaultCommand(
        subsystem,
        FunctionalCommand(
            [&initializeOrder, subsystem] {
              initializeOrder.emplace_back(subsystem);
            },
            [] {}, [](bool) {}, [] { return false; }, {subsystem}));
  }

  scheduler.Run();

  ASSERT_EQ(initializeOrder.size(), registrationOrder.size());
  EXPECT_TRUE(std::equal(initializeOrder.begin(), initializeOrder.end(),
                         registrationOrder.begin()));
  scheduler.CancelAll();
}

TEST_F(JavaSchedulerParityTest,
       InterruptedCommandsEndInIncomingRequirementOrder) {
  CommandScheduler scheduler = GetScheduler();
  std::vector<std::string> endOrder;
  TestSubsystem firstHeldSubsystem;
  TestSubsystem secondHeldSubsystem;
  FunctionalCommand firstHolder{
      [] {}, [] {}, [&endOrder](bool) { endOrder.emplace_back("first"); },
      [] { return false; }, {&firstHeldSubsystem}};
  FunctionalCommand secondHolder{
      [] {}, [] {}, [&endOrder](bool) { endOrder.emplace_back("second"); },
      [] { return false; }, {&secondHeldSubsystem}};
  FunctionalCommand incoming{[] {}, [] {}, [](bool) {}, [] { return false; },
                             {&secondHeldSubsystem, &firstHeldSubsystem}};

  scheduler.Schedule(&firstHolder);
  scheduler.Schedule(&secondHolder);
  scheduler.Schedule(&incoming);

  EXPECT_EQ(endOrder, (std::vector<std::string>{"second", "first"}));
  scheduler.CancelAll();
}

TEST_F(JavaSchedulerParityTest, ScheduleDuringRunLoopInitializesAfterLoop) {
  CommandScheduler scheduler = GetScheduler();
  std::vector<std::string> events;
  FunctionalCommand scheduledFromLoop{
      [&events] { events.emplace_back("deferred initialize"); }, [] {},
      [](bool) {}, [] { return false; }};
  RunCommand deferredScheduler{[&events, &scheduler, &scheduledFromLoop] {
    events.emplace_back("first execute");
    scheduler.Schedule(&scheduledFromLoop);
  }};
  RunCommand laterCommand{
      [&events] { events.emplace_back("second execute"); }};

  scheduler.Schedule(&deferredScheduler);
  scheduler.Schedule(&laterCommand);
  scheduler.Run();

  EXPECT_EQ(events, (std::vector<std::string>{"first execute", "second execute",
                                              "deferred initialize"}));
  scheduler.CancelAll();
}

TEST_F(JavaSchedulerParityTest, CancelDuringRunLoopEndsAfterLoop) {
  CommandScheduler scheduler = GetScheduler();
  std::vector<std::string> events;
  FunctionalCommand cancelledFromLoop{
      [] {}, [&events] { events.emplace_back("cancelled execute"); },
      [&events](bool interrupted) {
        events.emplace_back(interrupted ? "cancelled end interrupted"
                                        : "cancelled end");
      },
      [] { return false; }};
  RunCommand canceller{[&events, &scheduler, &cancelledFromLoop] {
    events.emplace_back("canceller execute");
    scheduler.Cancel(&cancelledFromLoop);
  }};

  scheduler.Schedule(&canceller);
  scheduler.Schedule(&cancelledFromLoop);
  scheduler.Run();

  EXPECT_EQ(events,
            (std::vector<std::string>{"canceller execute", "cancelled execute",
                                      "cancelled end interrupted"}));
  EXPECT_FALSE(scheduler.IsScheduled(&cancelledFromLoop));
  scheduler.CancelAll();
}
