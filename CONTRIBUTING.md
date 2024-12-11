# Contributing to mhsboat_ctrl

This guide will help you understand our code style, paradigms, and best practices to ensure consistency and maintainability.

## Code Style

### General Guidelines
- Follow PEP 8 for Python code style.
- Use 4 spaces for indentation.
- Keep lines under 80 characters.
- Use meaningful variable and function names.
- Add docstrings to all public modules, classes, and functions.

### Imports
- Group imports into two sections: library imports and local imports.
- Use absolute imports for clarity.

e.g.
```python
import os
import sys
import numpy as np
import rclpy

from mhsboat_ctrl.tasks import Task
```

### Typing
- Use type hints for function signatures and variable declarations.
- Use `Optional` for variables that can be `None`.
- Write docstrings for functions that describe everything the function does and requires.

### Comments
- Use comments to explain the purpose of complex code blocks.
- Use `# TODO:` to indicate areas that need improvement or additional features.

## Paradigms

### Object-Oriented Programming
- Use classes to encapsulate related data and functions.
- Inherit from base classes to promote code reuse and maintainability.
- Use properties to provide controlled access to class attributes.

### Modularity
- Organize code into modules and packages to promote separation of concerns.
- Each module should have a single responsibility.
- Use the `Task` class for defining tasks and ensure each task is in its own file.

## Best Practices

### Testing
- Write unit tests for all new features and bug fixes.
- Use the `pytest` framework for testing.
- Ensure tests cover edge cases and potential failure points.

### Documentation
- Update the README.md and other relevant documentation with any new features or changes.
- Use docstrings to document functions, classes, and modules.

### Version Control
- Use meaningful commit messages that describe the changes made.
- Create a new branch for each feature or bug fix.
- Submit pull requests for review before merging into the main branch.