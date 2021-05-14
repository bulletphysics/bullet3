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
"""Python benchmarking utilities.

Example usage:
  import benchmark

  @benchmark.register
  def my_benchmark(state):
      ...  # Code executed outside `while` loop is not timed.

      while state:
        ...  # Code executed within `while` loop is timed.

  if __name__ == '__main__':
    benchmark.main()
"""

from absl import app
from benchmark import _benchmark

__all__ = [
    "register",
    "main",
]

__version__ = "0.1.0"


def register(f=None, *, name=None):
  if f is None:
    return lambda f: register(f, name=name)
  if name is None:
    name = f.__name__
  _benchmark.RegisterBenchmark(name, f)
  return f


def _flags_parser(argv):
  argv = _benchmark.Initialize(argv)
  return app.parse_flags_with_usage(argv)


def _run_benchmarks(argv):
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')
  return _benchmark.RunSpecifiedBenchmarks()


def main(argv=None):
  return app.run(_run_benchmarks, argv=argv, flags_parser=_flags_parser)
