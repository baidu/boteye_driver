/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <driver/helper/ring_buffer.h>
#include <iostream>

using XPDRIVER::RingBuffer;

void print_ring_buffer(const RingBuffer<uint64_t>& ring_buffer) {
  std::cout << "ring_buffer size = " << ring_buffer.size() << ":\n";
  if (!ring_buffer.empty()) {
    std::cout << "Contains elements: ";
    for (size_t index = 0; index < ring_buffer.size(); ++index) {
      std::cout << ring_buffer[index] << " ";
    }
    std::cout << "\n"
              << "front:       " << ring_buffer.front() << "\n"
              << "back:        " << ring_buffer.back() << "\n";
  }
  if (ring_buffer.size() > 1) {
    std::cout << "2nd-to-last: " << ring_buffer.second_to_last() << "\n";
  }
}

int main(int argc, char** argv) {
  RingBuffer<uint64_t>  ring_buffer(4);
  print_ring_buffer(ring_buffer);

  for (uint64_t i = 10; i < 16; ++i) {
    ring_buffer.push_back(i);
    print_ring_buffer(ring_buffer);
  }
  while (!ring_buffer.empty()) {
    ring_buffer.pop_front();
    print_ring_buffer(ring_buffer);
  }
  for (uint64_t i = 70; i < 78; ++i) {
    ring_buffer.push_back(i);
    print_ring_buffer(ring_buffer);
  }
  return 0;
}
