# MAPF simulation on Isaac sim

this project implement path planning using cbs& a*manhattan and path following with ros2 simulation on Isaac sim 


## a*manhattan
the a* manhattan path finfing is similar to original a* but then result of path will provide manhattan distance view by implement new g score, a* manhattan is use in this project because the result of a* manhattan path is require less robot rotation.

## uneven a*
the uneven a* is similar to the standard a*, but it differs in the implementation of the g-cost. the g cost in uneven a* is implement by config g cost by each action of next path that current path must follow (straight, rotate_left, rotate_right) 

## Authors

- [@aelde_eggs](suppalerk1561@gmail.com)

