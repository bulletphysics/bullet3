# Copyright 2020 Google Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Example of Python using C++ benchmark framework.

To run this example, you must first install the `benchmark` Python package.

To install using `setup.py`, download and extract the `benchmark` source.
In the extracted directory, execute:
  python setup.py install
"""

import benchmark


@benchmark.register
def empty(state):
  while state:
    pass


@benchmark.register
def sum_million(state):
  while state:
    sum(range(1_000_000))


@benchmark.register
def skipped(state):
  if True:  # Test some predicate here.
    state.skip_with_error('some error')
    return  # NOTE: You must explicitly return, or benchmark will continue.

  ...  # Benchmark code would be here.


if __name__ == '__main__':
  benchmark.main()
