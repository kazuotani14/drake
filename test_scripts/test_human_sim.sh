echo "Starting Drake visualizer"
../bazel-bin/tools/drake_visualizer --script \
    ~/icub/drake-distro/drake/multibody/rigid_body_plant/visualization/contact_viz.py &
sleep 5
echo "Starting human model simulation"
../bazel-bin/drake/examples/human_model/human_passive_simulation
