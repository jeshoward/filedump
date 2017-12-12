/*
 * @copyright Copyright (C) 2017, Jessica Howard
 * @author Jessica Howard
 * @file turtlebot_rrt/test/vertex_test.cc
 *
 * @brief Unit tests for the RRT path planning algorithm
 *
 * @license 3-Clause BSD License
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 #include "mock_vertex.h"
 #include "gmock/gmock.h"
 #include "gtest/gtest.h"

 using ::testing::AtLeast;

 TEST(VertexTest, SingleVertex) {
  MockVertex vertex(5.0, 7.5, 3, 2);
  std::pair<float, float> location(5.0, 7.5);

  // test get location
  ON_CALL(vertex, get_location())
    .Field(&std::vector::first, FloatEq(5.0));
    .Field(&std::vector::second, FloatEq(7.5));

  // test get index
  ON_CALL(vertex, get_index())
    .WillByDefault(Return(3));

  // test get_parent
  ON_CALL(vertex, get_parent())
    .WillByDefault(Return(2));

  // use setter methods and retest
  ON_CALL(vertex, set_location(3.0, 6.23))
    .WillByDefault(Return());

  ON_CALL(vertex, set_index(12))
    .WillByDefault(Return());

  ON_CALL(vertex, set_parent(1))
    .WillByDefault(Return());

  // test get location
  ON_CALL(vertex, get_location())
    .Field(&std::vector::first, FloatEq(3.0));
    .Field(&std::vector::second, FloatEq(6.23));

  // test get index
  ON_CALL(vertex, get_index())
    .WillByDefault(Return(12));

  // test get_parent
  ON_CALL(vertex, get_parent())
    .WillByDefault(Return(1));
 }
