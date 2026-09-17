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

// The activation controller only ever compiles against the handle API of the
// distro it is built on, so the branch that distro does not use would otherwise
// go untested. These fakes stand in for both LoanedCommandInterface shapes so
// that every distro exercises both.
//
// Humble EOL: delete this file with the shims it covers.

#include <gtest/gtest.h>

#include <optional>

#include <robotiq_controllers/ros2_control_compat.hpp>

namespace robotiq_controllers::test {

/// Humble (ros2_control 2.x): writes cannot fail, reads always yield a value.
class OldApiHandle
{
public:
   void set_value(double value) { value_ = value; }
   double get_value() const { return value_; }

private:
   double value_ = 0.0;
};

/// Jazzy 4.x and later: writes report success, reads may come back empty.
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

} // namespace robotiq_controllers::test
