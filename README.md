# robot_env
Robosuite + Indy7 + GIC all combined together

## Installation
```source
git clone --recurse-submodules git@github.com:HorowitzManipulationLab/robot_env.git 
```

**NOTE**  
`--recurse-submodules` key is a must to run the package.

### Install Dependencies
``` source
cd your_directory/robot_env/robosuite
pip install -e .
```
```
cd your_directory/robot_env/robosuite_models
pip install -e .
```
Or any other build from source codes provided by the original authors.

## Running

cd to
```source
cd /your_directory/robot_env/scripts
```

**NOTE** \
You should run the main codes on the `scripts` directory, not `robot_env` directory, otherwise, `robosuite` folder name will be recognized as a package itself and make directory collision issue. Otherwise, you can rename the `robosuite` folder to something else, like `robosuite_source` and then re-build it. 

Run
```source
python env_test_real.py
```