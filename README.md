# MAPF project
this pj use cbs(astar) for control(planner ,follower) and simulation on isaac sim.

## about this project
how pj show how to plan multi robot paths and control them in ros and simulation on isaac.

## Installation
install [python](https://www.python.org/downloads/),
install [ros-humble](https://docs.ros.org/en/humble/Installation.html) because this project requires it. The project also has custom messages and services, so use **colcon** build to complete the setup.

## Usage
**all maps** can be found in **/mapf_isaac/_map.py(np.array map)**  
each np.array map refferent each node in isaac sim(pic below)  
<img src="/z_img_readme/image.png" alt="drawing" width="600"/>
these map represent each node in isaac 
to create your custom map create new variable in **/mapf_isaac/_map.py** then use **_map_generate.py** and **_rotate_map.py** to make map like this
```python
import foobar

# returns 'words'
foobar.pluralize('word')

# returns 'geese'
foobar.pluralize('goose')

# returns 'phenomenon'
foobar.singularize('phenomena')
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)