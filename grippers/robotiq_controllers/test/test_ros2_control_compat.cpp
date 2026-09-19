// Copyright (c) 2026 Robotiq
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// The controllers only ever compile against the handle and realtime-publisher
// APIs of the distro they are built on, so the branch that distro does not use
// would otherwise go untested. These fakes stand in for both shapes of each so
// that every distro exercises both.
//
// Humble EOL: delete this file with the shims it covers.

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include <robotiq_controllers/ros2_control_compat.hpp>

namespace robotiq_controllers::test {

// Humble: writes cannot fail, reads always yield a value.
class OldApiHandle
{
public:
   void set_value(double value) { value_ = value; }
   double get_value() const { return value_; }

private:
   double value_ = 0.0;
};

// Jazzy and later: writes report success, reads may come back empty.
class NewApiHandle
{
public:
   bool set_value(double value)
   {
      if(!writable_)
      {
         return false;
      }
      value_ = value;
      return true;
   }

   std::optional<double> get_optional() const { return readable_ ? std::optional<double>{value_} : std::nullopt; }

   void setWritable(bool writable) { writable_ = writable; }
   void setReadable(bool readable) { readable_ = readable; }

private:
   double value_ = 0.0;
   bool writable_ = true;
   bool readable_ = true;
};

// Humble: publishing goes through the lock and msg_.
class OldApiPublisher
{
public:
   bool tryPublish(const std::string& message)
   {
      if(!lockable_)
      {
         return false;
      }
      published_.push_back(message);
      return true;
   }

   void setLockable(bool lockable) { lockable_ = lockable; }
   const std::vector<std::string>& published() const { return published_; }

private:
   bool lockable_ = true;
   std::vector<std::string> published_;
};

// Jazzy and later: one non-blocking call.
class NewApiPublisher
{
public:
   bool try_publish(const std::string& message)
   {
      if(!lockable_)
      {
         return false;
      }
      published_.push_back(message);
      return true;
   }

   void setLockable(bool lockable) { lockable_ = lockable; }
   const std::vector<std::string>& published() const { return published_; }

private:
   bool lockable_ = true;
   std::vector<std::string> published_;
};

TEST(TestRos2ControlCompat, detects_each_handle_shape)
{
   EXPECT_FALSE(compat::detail::HasGetOptional<OldApiHandle>::value);
   EXPECT_FALSE(compat::detail::SetValueReturnsBool<OldApiHandle>::value);
   EXPECT_TRUE(compat::detail::HasGetOptional<NewApiHandle>::value);
   EXPECT_TRUE(compat::detail::SetValueReturnsBool<NewApiHandle>::value);
}

TEST(TestRos2ControlCompat, old_api_round_trips_and_always_reports_success)
{
   OldApiHandle handle;

   EXPECT_TRUE(compat::setValue(handle, 4.25));
   EXPECT_EQ(compat::getValue(handle), std::optional<double>{4.25});
}

TEST(TestRos2ControlCompat, new_api_round_trips)
{
   NewApiHandle handle;

   EXPECT_TRUE(compat::setValue(handle, 4.25));
   EXPECT_EQ(compat::getValue(handle), std::optional<double>{4.25});
}

TEST(TestRos2ControlCompat, new_api_propagates_write_failure)
{
   NewApiHandle handle;
   handle.setWritable(false);

   EXPECT_FALSE(compat::setValue(handle, 4.25));
}

TEST(TestRos2ControlCompat, new_api_propagates_empty_read)
{
   NewApiHandle handle;
   handle.setReadable(false);

   EXPECT_EQ(compat::getValue(handle), std::nullopt);
}

TEST(TestRos2ControlCompat, detects_each_publisher_shape)
{
   EXPECT_FALSE((compat::detail::HasTryPublish<OldApiPublisher, std::string>::value));
   EXPECT_TRUE((compat::detail::HasTryPublish<NewApiPublisher, std::string>::value));
}

TEST(TestRos2ControlCompat, old_publisher_api_publishes_the_message)
{
   OldApiPublisher publisher;

   EXPECT_TRUE(compat::tryPublish(publisher, std::string{"status"}));
   EXPECT_EQ(publisher.published(), std::vector<std::string>{"status"});
}

TEST(TestRos2ControlCompat, old_publisher_api_drops_the_message_when_the_buffer_is_taken)
{
   OldApiPublisher publisher;
   publisher.setLockable(false);

   EXPECT_FALSE(compat::tryPublish(publisher, std::string{"status"}));
   EXPECT_TRUE(publisher.published().empty());
}

TEST(TestRos2ControlCompat, new_publisher_api_publishes_the_message)
{
   NewApiPublisher publisher;

   EXPECT_TRUE(compat::tryPublish(publisher, std::string{"status"}));
   EXPECT_EQ(publisher.published(), std::vector<std::string>{"status"});
}

TEST(TestRos2ControlCompat, new_publisher_api_propagates_a_refused_publish)
{
   NewApiPublisher publisher;
   publisher.setLockable(false);

   EXPECT_FALSE(compat::tryPublish(publisher, std::string{"status"}));
   EXPECT_TRUE(publisher.published().empty());
}

} // namespace robotiq_controllers::test
