/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2020 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/// @author Ian Purvis <ian@purvisresearch.com>

#include "Benchmarks/BenchmarkDemo.h"
#include "CommonInterfaces/CommonExampleInterface.h"
#include "Headless/HeadlessInterfaceHelper.h"
#include <benchmark/benchmark.h>
#include <regex>
#include <string>

extern bool gDisableDeactivation;

const char* HELP = R"(NAME:

    App_Benchmarks - bullet physics benchmarks

SYNOPSIS:

    App_Benchmarks [options]

OPTIONS

    --start=<n>
        Start benchmarking after <n> steps. Default is 0.

    --steps=<n>
        Run the benchmark for <n> steps. Default is 200.

    --benchmark_list_tests={true|false}
    --benchmark_filter=<regex>
    --benchmark_min_time=<min_time>
    --benchmark_repetitions=<num_repetitions>
    --benchmark_report_aggregates_only={true|false}
    --benchmark_display_aggregates_only={true|false}
    --benchmark_format=<console|json|csv>
    --benchmark_out=<filename>
    --benchmark_out_format=<json|console|csv>
    --benchmark_color={auto|true|false}
    --benchmark_counters_tabular={true|false}
    --v=<verbosity>
    --help
)";

const struct Benchmark
{
	int id;
	const char* name;
} BENCHMARKS[]{
	// Benchmark a stack of 3000 boxes. It will stress the collision detection,
	// a specialized box-box implementation based on the separating axis test,
	// and the constraint solver.
	{1, "3000 Boxes"},

	// Benchmark a stack of 1000 boxes. It will stress the collision detection,
	// a specialized box-box implementation based on the separating axis test,
	// and the constraint solver.
	{2, "1000 Stack"},

	// Benchmark the performance of the ragdoll constraints, btHingeConstraint
	// and btConeTwistConstraint, in addition to capsule collision detection.
	{3, "Ragdolls"},

	// Benchmark the performance and stability of rigid bodies using btConvexHullShape.
	{4, "Convex Stack"},

	// Benchmark the performance and stability of rigid bodies using primitive
	// collision shapes (btSphereShape, btBoxShape), resting on a triangle mesh,
	// btBvhTriangleMeshShape.
	{5, "Prim vs Mesh"},

	// Benchmark the performance and stability of rigid bodies using convex hull
	// collision shapes (btConvexHullShape), resting on a triangle mesh,
	// btBvhTriangleMeshShape.
	{6, "Convex vs Mesh"},

	// Benchmark the performance of the btCollisionWorld::rayTest.
	// Note that currently the rays are not rendered.
	{7, "Raycast"},

	// Benchmark the performance of the convex hull primitive.
	{8, "Convex Pack"}};

const float TIMESTEP = 1.0f / 60.0f;

void run(benchmark::State& state, int id, int start, int steps)
{
	HeadlessInterfaceHelper interfaceHelper;
	CommonExampleOptions* options;
	CommonExampleInterface* demo;
	int benchmarkedFrames = 0;

	gDisableDeactivation = true;

	for (auto _ : state)
	{
		state.PauseTiming();
		options = new CommonExampleOptions(&interfaceHelper, id);
		demo = BenchmarkCreateFunc(*options);
		demo->initPhysics();
		for (int f = 0; f < start - 1; f++)
		{
			demo->stepSimulation(TIMESTEP);
		}
		state.ResumeTiming();

		for (int f = 0; f < steps; f++)
		{
			demo->stepSimulation(TIMESTEP);
			benchmarkedFrames++;
		}

		state.PauseTiming();
		demo->exitPhysics();
		delete demo;
		delete options;
		state.ResumeTiming();
	}

	state.counters["fps"] = benchmark::Counter(benchmarkedFrames, benchmark::Counter::kIsRate);
	state.counters["start"] = start;
	state.counters["steps"] = steps;
}

int main(int argc, char** argv)
{
	int start = 0;
	int steps = 200;

	std::regex startPattern("^--start=(.*)$");
	std::regex stepsPattern("^--steps=(.*)$");
	std::regex helpPattern("^--help$");
	std::cmatch match;
	for (int i = 0; i < argc; i++)
	{
		if (std::regex_match(argv[i], match, startPattern))
		{
			start = std::stoi(match[1]);
		}
		else if (std::regex_match(argv[i], match, stepsPattern))
		{
			steps = std::stoi(match[1]);
		}
		else if (std::regex_match(argv[i], match, helpPattern))
		{
			printf("%s\n", HELP);
			exit(0);
		}
	}

	for (auto& b : BENCHMARKS)
	{
		benchmark::RegisterBenchmark(b.name, &run, b.id, start, steps)->Unit(benchmark::kMillisecond);
	}
	benchmark::Initialize(&argc, argv);
	benchmark::RunSpecifiedBenchmarks();
}
