// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "CommandTestBase.hpp"
#include "wpi/commands2/FunctionalCommand.hpp"
#include "wpi/commands2/RunCommand.hpp"

using namespace wpi::cmd;

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

TEST_CASE_METHOD(JavaSchedulerParityTest,
                 "JavaSchedulerParityTest PeriodicRunsInRegistrationOrder",
                 "[commandsv2][command]") {
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

  REQUIRE(periodicOrder.size() == registrationOrder.size());
  CHECK(std::equal(periodicOrder.begin(), periodicOrder.end(),
                   registrationOrder.begin()));
}

TEST_CASE_METHOD(JavaSchedulerParityTest,
                 "JavaSchedulerParityTest ExecuteRunsInSchedulingOrder",
                 "[commandsv2][command]") {
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

  REQUIRE(executeOrder.size() == schedulingOrder.size());
  CHECK(std::equal(executeOrder.begin(), executeOrder.end(),
                   schedulingOrder.begin()));
  scheduler.CancelAll();
}

TEST_CASE_METHOD(JavaSchedulerParityTest,
                 "JavaSchedulerParityTest DefaultCommandsScheduleInRegistrationOrder",
                 "[commandsv2][command]") {
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

  REQUIRE(initializeOrder.size() == registrationOrder.size());
  CHECK(std::equal(initializeOrder.begin(), initializeOrder.end(),
                   registrationOrder.begin()));
  scheduler.CancelAll();
}

TEST_CASE_METHOD(
    JavaSchedulerParityTest,
    "JavaSchedulerParityTest InterruptedCommandsEndInIncomingRequirementOrder",
    "[commandsv2][command]") {
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

  CHECK(endOrder == std::vector<std::string>{"second", "first"});
  scheduler.CancelAll();
}

TEST_CASE_METHOD(JavaSchedulerParityTest,
                 "JavaSchedulerParityTest ScheduleDuringRunLoopInitializesAfterLoop",
                 "[commandsv2][command]") {
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

  CHECK(events == std::vector<std::string>{"first execute", "second execute",
                                           "deferred initialize"});
  scheduler.CancelAll();
}

TEST_CASE_METHOD(JavaSchedulerParityTest,
                 "JavaSchedulerParityTest CancelDuringRunLoopEndsAfterLoop",
                 "[commandsv2][command]") {
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

  CHECK(events ==
        std::vector<std::string>{"canceller execute", "cancelled execute",
                                 "cancelled end interrupted"});
  CHECK_FALSE(scheduler.IsScheduled(&cancelledFromLoop));
  scheduler.CancelAll();
}
