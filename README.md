```bash
echo "deb [trusted=yes] https://github.com/ntnu-arl/vectornav/raw/focal-noetic/ ./" | sudo tee /etc/apt/sources.list.d/ntnu-arl_vectornav.list
echo "yaml https://github.com/ntnu-arl/vectornav/raw/focal-noetic/local.yaml noetic" | sudo tee /etc/ros/rosdep/sources.list.d/1-ntnu-arl_vectornav.list
```
