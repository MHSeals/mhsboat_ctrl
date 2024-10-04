# mhsboat_ctrl next

This is my concept for the 2024/2025 MHS boat control system

Task file live in the `mhsboat_ctrl/tasks` directory. Each task is a separate class that inherits from the `Task` class in `mhsboat_ctrl/tasks/task.py`. The `Task` class has a `run` method that is called by the main loop of the control system. The `run` method should return a `Task` object that is the next task to be run. The `Task` class also has a `search` method which searches for the next task to run. The `search` method should return either `None` if it isn't found, or a tuple `(x, y)` where `x` and `y` are the coordinates of where the task was found.

Tasks need to be added to the `mhsboat_ctrl/tasks/__init__.py` file in order to be found by the control system.

```python
...
    self.add_task(ExampleTask())
...
```

The main loop of the control system is in `mhsboat_ctrl/main.py`. It is a simple loop that searches for the next task to run, runs it, and then repeats.