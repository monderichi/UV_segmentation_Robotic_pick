#!/usr/bin/env python3
"""
ROS2 Launch File Analyzer

Analyzes Python launch files to extract:
- Launch arguments with defaults
- Nodes (package, executable, name, parameters)
- Included launch files
- Timer actions and delayed execution
- Conditional launching

Usage:
    python3 analyze_launch.py <path_to_launch_file>
    python3 analyze_launch.py <path_to_launch_file> --format json
"""

import ast
import sys
import re
from pathlib import Path
from typing import Dict, List, Any, Optional
import argparse
import json


class LaunchAnalyzer(ast.NodeVisitor):
    """AST visitor to analyze ROS2 launch files."""
    
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.launch_args: List[Dict[str, Any]] = []
        self.nodes: List[Dict[str, Any]] = []
        self.included_launches: List[Dict[str, Any]] = []
        self.timer_actions: List[Dict[str, Any]] = []
        self.has_opaque_function: bool = False
        self.imports: Dict[str, str] = {}
        self.opaque_function_body: Optional[ast.FunctionDef] = None
        
    def analyze(self) -> Dict[str, Any]:
        """Parse and analyze the launch file."""
        try:
            with open(self.file_path, 'r') as f:
                tree = ast.parse(f.read())
        except SyntaxError as e:
            return {"error": f"Syntax error: {e}"}
        except FileNotFoundError:
            return {"error": f"File not found: {self.file_path}"}
            
        self.visit(tree)
        return self._get_results()
    
    def visit_Import(self, node):
        """Track imports."""
        for alias in node.names:
            name = alias.name
            asname = alias.asname or name
            if '.' in name:
                base = name.split('.')[0]
                self.imports[asname] = base
            else:
                self.imports[asname] = name
        self.generic_visit(node)
    
    def visit_ImportFrom(self, node):
        """Track from imports."""
        module = node.module or ""
        for alias in node.names:
            name = alias.name
            asname = alias.asname or name
            self.imports[asname] = f"{module}.{name}"
        self.generic_visit(node)
    
    def visit_Call(self, node):
        """Analyze function calls for launch constructs."""
        func_name = self._get_func_name(node.func)
        
        if func_name == "DeclareLaunchArgument":
            self._parse_declare_launch_argument(node)
        elif func_name == "Node":
            self._parse_node(node)
        elif func_name == "IncludeLaunchDescription":
            self._parse_include_launch(node)
        elif func_name == "TimerAction":
            self._parse_timer_action(node)
        elif func_name == "OpaqueFunction":
            self.has_opaque_function = True
        elif func_name == "ExecuteProcess":
            self._parse_execute_process(node)
            
        self.generic_visit(node)
    
    def visit_FunctionDef(self, node):
        """Track function definitions."""
        if node.name in ["launch_setup", "generate_launch_description", "configure_setup"]:
            if node.name in ["launch_setup", "configure_setup"]:
                self.opaque_function_body = node
            # Analyze the function body for nodes
            for stmt in ast.walk(node):
                if isinstance(stmt, ast.Call):
                    func_name = self._get_func_name(stmt.func)
                    if func_name == "Node":
                        self._parse_node(stmt)
                    elif func_name == "IncludeLaunchDescription":
                        self._parse_include_launch(stmt)
                    elif func_name == "TimerAction":
                        self._parse_timer_action(stmt)
        self.generic_visit(node)
    
    def _get_func_name(self, func) -> str:
        """Extract function name from AST node."""
        if isinstance(func, ast.Name):
            return func.id
        elif isinstance(func, ast.Attribute):
            return func.attr
        return ""
    
    def _parse_declare_launch_argument(self, node):
        """Parse DeclareLaunchArgument call."""
        arg_info = {"name": "", "default_value": None, "description": ""}
        
        # Parse positional arguments
        if node.args:
            if len(node.args) > 0:
                arg_info["name"] = self._extract_value(node.args[0])
            if len(node.args) > 1:
                arg_info["default_value"] = self._extract_value(node.args[1])
        
        # Parse keyword arguments
        for kw in node.keywords:
            if kw.arg == "name":
                arg_info["name"] = self._extract_value(kw.value)
            elif kw.arg == "default_value":
                arg_info["default_value"] = self._extract_value(kw.value)
            elif kw.arg == "description":
                arg_info["description"] = self._extract_value(kw.value)
        
        if arg_info["name"]:
            self.launch_args.append(arg_info)
    
    def _parse_node(self, node):
        """Parse Node call."""
        node_info = {
            "package": "",
            "executable": "",
            "name": None,
            "output": None,
            "parameters": [],
            "arguments": [],
            "remappings": [],
            "has_condition": False
        }
        
        for kw in node.keywords:
            if kw.arg == "package":
                node_info["package"] = self._extract_value(kw.value)
            elif kw.arg == "executable":
                node_info["executable"] = self._extract_value(kw.value)
            elif kw.arg == "name":
                node_info["name"] = self._extract_value(kw.value)
            elif kw.arg == "output":
                node_info["output"] = self._extract_value(kw.value)
            elif kw.arg == "parameters":
                node_info["parameters"] = self._extract_parameters(kw.value)
            elif kw.arg == "arguments":
                node_info["arguments"] = self._extract_list(kw.value)
            elif kw.arg == "remappings":
                node_info["remappings"] = self._extract_list(kw.value)
            elif kw.arg == "condition":
                node_info["has_condition"] = True
        
        self.nodes.append(node_info)
    
    def _parse_include_launch(self, node):
        """Parse IncludeLaunchDescription call."""
        include_info = {
            "path": None,
            "launch_arguments": {},
            "has_condition": False
        }
        
        for kw in node.keywords:
            if kw.arg == "launch_description_source":
                include_info["path"] = self._extract_launch_path(kw.value)
            elif kw.arg == "launch_arguments":
                include_info["launch_arguments"] = self._extract_dict(kw.value)
            elif kw.arg == "condition":
                include_info["has_condition"] = True
        
        self.included_launches.append(include_info)
    
    def _parse_timer_action(self, node):
        """Parse TimerAction call."""
        timer_info = {
            "period": None,
            "actions": []
        }
        
        for kw in node.keywords:
            if kw.arg == "period":
                timer_info["period"] = self._extract_value(kw.value)
            elif kw.arg == "actions":
                # Try to find what actions are inside
                for item in ast.walk(kw.value):
                    if isinstance(item, ast.Call):
                        func_name = self._get_func_name(item.func)
                        timer_info["actions"].append(func_name)
        
        self.timer_actions.append(timer_info)
    
    def _parse_execute_process(self, node):
        """Parse ExecuteProcess call."""
        exec_info = {"cmd": [], "shell": False}
        
        for kw in node.keywords:
            if kw.arg == "cmd":
                exec_info["cmd"] = self._extract_list(kw.value)
            elif kw.arg == "shell":
                exec_info["shell"] = self._extract_value(kw.value)
        
        # Add as a pseudo-node for reference
        if exec_info["cmd"]:
            self.nodes.append({
                "package": "EXECUTE_PROCESS",
                "executable": str(exec_info["cmd"][:2]) if len(exec_info["cmd"]) > 1 else str(exec_info["cmd"]),
                "name": None,
                "output": None,
                "parameters": [],
                "arguments": exec_info["cmd"],
                "remappings": [],
                "has_condition": False
            })
    
    def _extract_value(self, node) -> Any:
        """Extract a value from an AST node."""
        if isinstance(node, ast.Constant):
            return node.value
        elif isinstance(node, ast.Str):  # Python 3.7 compatibility
            return node.s
        elif isinstance(node, ast.Num):  # Python 3.7 compatibility
            return node.n
        elif isinstance(node, ast.NameConstant):  # Python 3.7 compatibility
            return node.value
        elif isinstance(node, ast.Name):
            return f"<{node.id}>"
        elif isinstance(node, ast.Attribute):
            return f"<{node.attr}>"
        elif isinstance(node, ast.JoinedStr):
            parts = []
            for value in node.values:
                parts.append(str(self._extract_value(value)))
            return "".join(parts)
        elif isinstance(node, ast.BinOp):
            if isinstance(node.op, ast.Add):
                return f"{self._extract_value(node.left)} + {self._extract_value(node.right)}"
        return "<complex>"
    
    def _extract_list(self, node) -> List[Any]:
        """Extract a list from an AST node."""
        if isinstance(node, ast.List):
            return [self._extract_value(elt) for elt in node.elts]
        return ["<list>"]
    
    def _extract_dict(self, node) -> Dict[str, Any]:
        """Extract a dict from an AST node."""
        result = {}
        if isinstance(node, ast.Dict):
            for k, v in zip(node.keys, node.values):
                key = self._extract_value(k)
                result[str(key)] = self._extract_value(v)
        return result
    
    def _extract_parameters(self, node) -> List[Any]:
        """Extract parameters list."""
        params = []
        if isinstance(node, ast.List):
            for elt in node.elts:
                if isinstance(elt, ast.Dict):
                    params.append(self._extract_dict(elt))
                elif isinstance(elt, ast.Name):
                    params.append(f"<{elt.id}>")
                else:
                    params.append(self._extract_value(elt))
        return params
    
    def _extract_launch_path(self, node) -> str:
        """Extract launch file path from PythonLaunchDescriptionSource."""
        if isinstance(node, ast.Call):
            # Check for PathJoinSubstitution
            if self._get_func_name(node.func) == "PathJoinSubstitution":
                for kw in node.keywords:
                    if kw.arg in ["FindPackageShare", "FindExecutable"]:
                        return f"<package>/{self._extract_value(kw.value)}"
            # Check for list
            if node.args and isinstance(node.args[0], ast.List):
                parts = self._extract_list(node.args[0])
                return "/".join(str(p) for p in parts)
        return "<unknown>"
    
    def _get_results(self) -> Dict[str, Any]:
        """Compile analysis results."""
        return {
            "file": self.file_path,
            "pattern": "OpaqueFunction" if self.has_opaque_function else "Direct",
            "launch_arguments": self.launch_args,
            "nodes": self.nodes,
            "included_launches": self.included_launches,
            "timer_actions": self.timer_actions,
            "summary": {
                "total_launch_args": len(self.launch_args),
                "total_nodes": len(self.nodes),
                "total_included_launches": len(self.included_launches),
                "total_timer_actions": len(self.timer_actions),
            }
        }


def format_output_text(results: Dict[str, Any]) -> str:
    """Format results as human-readable text."""
    lines = []
    lines.append("=" * 60)
    lines.append(f"ROS2 Launch File Analysis: {results['file']}")
    lines.append("=" * 60)
    lines.append(f"Pattern: {results['pattern']}")
    lines.append("")
    
    # Launch Arguments
    lines.append("-" * 40)
    lines.append("LAUNCH ARGUMENTS")
    lines.append("-" * 40)
    for arg in results['launch_arguments']:
        default = f" (default: {arg['default_value']})" if arg['default_value'] else ""
        lines.append(f"  {arg['name']}{default}")
        if arg['description']:
            lines.append(f"    Description: {arg['description']}")
    lines.append("")
    
    # Nodes
    lines.append("-" * 40)
    lines.append("NODES")
    lines.append("-" * 40)
    for node in results['nodes']:
        name_str = f" [{node['name']}]" if node['name'] else ""
        cond_str = " [CONDITIONAL]" if node['has_condition'] else ""
        lines.append(f"  {node['package']}/{node['executable']}{name_str}{cond_str}")
        if node['output']:
            lines.append(f"    Output: {node['output']}")
        if node['parameters']:
            lines.append(f"    Parameters: {node['parameters']}")
        if node['arguments']:
            lines.append(f"    Arguments: {node['arguments']}")
        if node['remappings']:
            lines.append(f"    Remappings: {node['remappings']}")
    lines.append("")
    
    # Included Launches
    if results['included_launches']:
        lines.append("-" * 40)
        lines.append("INCLUDED LAUNCH FILES")
        lines.append("-" * 40)
        for inc in results['included_launches']:
            cond_str = " [CONDITIONAL]" if inc['has_condition'] else ""
            lines.append(f"  {inc['path']}{cond_str}")
            if inc['launch_arguments']:
                lines.append(f"    Arguments: {inc['launch_arguments']}")
        lines.append("")
    
    # Timer Actions
    if results['timer_actions']:
        lines.append("-" * 40)
        lines.append("TIMER ACTIONS")
        lines.append("-" * 40)
        for timer in results['timer_actions']:
            lines.append(f"  Delay: {timer['period']}s")
            lines.append(f"    Actions: {timer['actions']}")
        lines.append("")
    
    # Summary
    lines.append("-" * 40)
    lines.append("SUMMARY")
    lines.append("-" * 40)
    summary = results['summary']
    lines.append(f"  Launch arguments: {summary['total_launch_args']}")
    lines.append(f"  Nodes: {summary['total_nodes']}")
    lines.append(f"  Included launches: {summary['total_included_launches']}")
    lines.append(f"  Timer actions: {summary['total_timer_actions']}")
    
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Analyze ROS2 Python launch files")
    parser.add_argument("launch_file", help="Path to the launch file to analyze")
    parser.add_argument("--format", choices=["text", "json"], default="text",
                        help="Output format (default: text)")
    args = parser.parse_args()
    
    analyzer = LaunchAnalyzer(args.launch_file)
    results = analyzer.analyze()
    
    if "error" in results:
        print(f"Error: {results['error']}", file=sys.stderr)
        sys.exit(1)
    
    if args.format == "json":
        print(json.dumps(results, indent=2))
    else:
        print(format_output_text(results))


if __name__ == "__main__":
    main()
