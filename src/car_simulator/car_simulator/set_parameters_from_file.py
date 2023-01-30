#Copied from https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/set_parameters_from_file.py

# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Module for the `SetParametersFromFile` action."""

from launch import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch_ros.utilities.evaluate_parameters import evaluate_parameter_dict
from launch_ros.utilities.normalize_parameters import normalize_parameter_dict
import ruamel.yaml
import collections


@expose_action('set_parameters_from_file')
class SetParametersFromFile(Action):
    """
    Action that sets parameters for all nodes in scope based on a given yaml file.

    e.g.
    ```python3
        LaunchDescription([
            ...,
            GroupAction(
                actions = [
                    ...,
                    SetParametersFromFile('path/to/file.yaml'),
                    ...,
                    Node(...),  // the params will be passed to this node
                    ...,
                ]
            ),
            Node(...),  // here it won't be passed, as it's not in the same scope
            ...
        ])
    ```
    ```xml
    <launch>
        <group>
            <set_parameters_from_file filename='/path/to/file.yaml'/>
            <node .../>    // Node in scope, params will be passed
        </group>
        <node .../>  // Node not in scope, params won't be passed
    </launch>

    ```
    """

    def __init__(
        self,
        filename: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """Create a SetParameterFromFile action."""
        super().__init__(**kwargs)
        self._input_file = normalize_to_list_of_substitutions(filename)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetParameterFromFile` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['filename'] = parser.parse_substitution(entity.get_attr('filename'))
        return cls, kwargs
    
    def flatten(self, d, parent_key='', sep='.'):
        items = []
        for k, v in d.items():
            new_key = parent_key + '.' + k if parent_key else k
            if isinstance(v, collections.MutableMapping):
                items.extend(self.flatten(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)

    def execute(self, context: LaunchContext):
        """Execute the action."""
        filename = perform_substitutions(context, self._input_file)
        with open(filename, "r") as stream:
            try:
                yaml = ruamel.yaml.safe_load(stream)
                if not isinstance(yaml, dict):
                    raise AttributeError('keys_exists() expects dict as first argument.')
                yaml_content = yaml
                try:
                    yaml_content = yaml_content['/**']
                    yaml_content = yaml_content['ros__parameters']
                except KeyError:
                    yaml_content = yaml_content
                param_dict = yaml_content
            except ruamel.yaml.YAMLError as error:
                print(error)
        
        param_dict = self.flatten(param_dict);
        
        param_dict = normalize_parameter_dict({tuple(normalize_to_list_of_substitutions(name)): value for name, value in param_dict.items()})
        
        eval_param_dict = evaluate_parameter_dict(context, param_dict)
        global_params = context.launch_configurations.get('ros_params', {})
        global_params.update(eval_param_dict)
        context.launch_configurations['ros_params'] = global_params
