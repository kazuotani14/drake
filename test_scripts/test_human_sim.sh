echo "Starting Drake visualizer"
../bazel-bin/tools/drake_visualizer&
sleep 2
echo "Starting human model simulation"
../bazel-bin/drake/examples/human_model/human_passive_simulation
