# PySwip Integration with ROS 2

## Overview

PySwip provides Python bindings for SWI-Prolog, enabling ROS 2 nodes to query and manipulate Prolog knowledge bases. This guide covers integration patterns for the Flyby-F11 autonomy stack.

## Installation

### System Requirements
- Ubuntu 22.04 (Jetson Orin NX)
- Python 3.10+
- SWI-Prolog 8.4+
- ROS 2 Humble

### Install SWI-Prolog
```bash
sudo apt-add-repository ppa:swi-prolog/stable
sudo apt update
sudo apt install swi-prolog
```

### Install PySwip
```bash
pip install pyswip
```

### Verify Installation
```python
from pyswip import Prolog

prolog = Prolog()
prolog.assertz("father(john, bob)")
prolog.assertz("father(john, alice)")

# Query
results = list(prolog.query("father(john, X)"))
print(results)
# Output: [{'X': 'bob'}, {'X': 'alice'}]
```

## Basic PySwip Usage

### Asserting Facts
```python
from pyswip import Prolog

prolog = Prolog()

# Assert simple facts
prolog.assertz("object(obj_1, person)")
prolog.assertz("object(obj_2, car)")
prolog.assertz("left_of(obj_1, obj_2)")
```

### Querying
```python
# Simple query
results = list(prolog.query("object(X, person)"))
print(results)  # [{'X': 'obj_1'}]

# Query with multiple variables
results = list(prolog.query("left_of(X, Y)"))
print(results)  # [{'X': 'obj_1', 'Y': 'obj_2'}]

# Query with conditions
results = list(prolog.query("object(X, person), left_of(X, Y)"))
```

### Retracting Facts
```python
# Retract specific fact
prolog.retract("object(obj_1, person)")

# Retract all matching facts
prolog.retractall("object(_, _)")
```

### Loading Prolog Files
```python
# Load ontology file
prolog.consult("/path/to/ontology.pl")

# Load multiple files
prolog.consult("/path/to/rules.pl")
prolog.consult("/path/to/facts.pl")
```

## ROS 2 Integration Patterns

### Pattern 1: Query Service

Create a ROS 2 service that executes Prolog queries:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pyswip import Prolog
from flyby_f11_interfaces.srv import PrologQuery

class PrologQueryService(Node):
    """
    ROS 2 service for executing Prolog queries.
    """

    def __init__(self):
        super().__init__('prolog_query_service')

        # Initialize Prolog engine
        self.prolog = Prolog()
        self.load_ontology()

        # Create service
        self.query_service = self.create_service(
            PrologQuery,
            '/reasoning/query',
            self.query_callback
        )

        self.get_logger().info('Prolog Query Service ready')

    def load_ontology(self):
        """Load Prolog ontology files."""
        ontology_files = [
            '/path/to/ontology.pl',
            '/path/to/rules.pl',
            '/path/to/spatial_reasoning.pl'
        ]

        for file_path in ontology_files:
            try:
                self.prolog.consult(file_path)
                self.get_logger().info(f'Loaded: {file_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load {file_path}: {e}')

    def query_callback(self, request, response):
        """
        Execute Prolog query and return results.

        Args:
            request: PrologQuery request with query string
            response: PrologQuery response with results

        Returns:
            response with success status and results
        """
        query_string = request.query

        try:
            # Execute query
            results = list(self.prolog.query(query_string))

            # Convert results to JSON-serializable format
            response.success = True
            response.results = str(results)
            response.num_results = len(results)

            self.get_logger().info(
                f'Query: {query_string} | Results: {len(results)}'
            )

        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().error(f'Query failed: {e}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PrologQueryService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Definition (PrologQuery.srv)
```
# Request
string query

---

# Response
bool success
string results
int32 num_results
string error_message
```

#### Client Example
```python
import rclpy
from rclpy.node import Node
from flyby_f11_interfaces.srv import PrologQuery

class PrologQueryClient(Node):
    def __init__(self):
        super().__init__('prolog_query_client')
        self.client = self.create_client(PrologQuery, '/reasoning/query')

        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for query service...')

    def query(self, query_string):
        """Execute Prolog query via service."""
        request = PrologQuery.Request()
        request.query = query_string

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            return eval(future.result().results)  # Convert string back to list
        else:
            self.get_logger().error(f'Query failed: {future.result().error_message}')
            return []

# Usage
client = PrologQueryClient()
results = client.query("object(X, person)")
print(results)  # [{'X': 'obj_1'}, {'X': 'obj_2'}]
```

### Pattern 2: Real-Time Fact Assertion

Update Prolog knowledge base in real-time from perception callbacks:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pyswip import Prolog
from flyby_f11_interfaces.msg import PerceptionObjectArray

class RealtimeKnowledgeBase(Node):
    """
    Maintains real-time Prolog knowledge base from perception.
    """

    def __init__(self):
        super().__init__('realtime_knowledge_base')

        # Initialize Prolog
        self.prolog = Prolog()
        self.prolog.consult('/path/to/ontology.pl')

        # Subscribe to perception
        self.perception_sub = self.create_subscription(
            PerceptionObjectArray,
            '/perception/objects',
            self.update_knowledge_base,
            10
        )

        # Timer for periodic cleanup
        self.cleanup_timer = self.create_timer(
            5.0,  # 5 seconds
            self.cleanup_old_facts
        )

        self.fact_timestamps = {}  # Track when facts were asserted

    def update_knowledge_base(self, msg):
        """
        Update knowledge base with new perception data.
        """
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Clear previous object facts
        self.prolog.retractall("detected_object(_, _, _, _, _)")

        # Assert new facts
        for obj in msg.objects:
            if obj.confidence < 0.5:
                continue

            fact = (
                f"detected_object("
                f"obj_{obj.track_id}, "
                f"{obj.class_name}, "
                f"[{obj.bbox.x_min}, {obj.bbox.y_min}], "
                f"{obj.confidence}, "
                f"{current_time}"
                f")"
            )

            self.prolog.assertz(fact)
            self.fact_timestamps[f"obj_{obj.track_id}"] = current_time

        self.get_logger().debug(f'Updated KB with {len(msg.objects)} objects')

    def cleanup_old_facts(self):
        """
        Remove facts older than threshold.
        """
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        threshold = 10.0  # 10 seconds

        for obj_id, timestamp in list(self.fact_timestamps.items()):
            if current_time - timestamp > threshold:
                # Retract old fact
                self.prolog.retractall(f"detected_object({obj_id}, _, _, _, _)")
                del self.fact_timestamps[obj_id]
                self.get_logger().debug(f'Removed stale fact: {obj_id}')

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeKnowledgeBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Pattern 3: Reasoning Action Server

Implement action server for complex reasoning tasks:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from pyswip import Prolog
from flyby_f11_interfaces.action import ExecuteReasoning

class ReasoningActionServer(Node):
    """
    Action server for executing complex reasoning tasks.
    """

    def __init__(self):
        super().__init__('reasoning_action_server')

        # Initialize Prolog
        self.prolog = Prolog()
        self.prolog.consult('/path/to/mission_planning.pl')

        # Create action server
        self.action_server = ActionServer(
            self,
            ExecuteReasoning,
            '/reasoning/execute',
            self.execute_callback
        )

        self.get_logger().info('Reasoning Action Server ready')

    def execute_callback(self, goal_handle):
        """
        Execute reasoning task.

        Args:
            goal_handle: Action goal with reasoning task

        Returns:
            Result with reasoning outcome
        """
        self.get_logger().info(f'Executing reasoning task: {goal_handle.request.task}')

        # Publish feedback
        feedback = ExecuteReasoning.Feedback()
        feedback.status = 'Querying Prolog...'
        goal_handle.publish_feedback(feedback)

        # Execute reasoning
        try:
            query = goal_handle.request.query
            results = list(self.prolog.query(query))

            # Process results
            if len(results) > 0:
                feedback.status = f'Found {len(results)} solutions'
                goal_handle.publish_feedback(feedback)

                # Return success
                goal_handle.succeed()
                result = ExecuteReasoning.Result()
                result.success = True
                result.solutions = str(results)
                return result
            else:
                feedback.status = 'No solutions found'
                goal_handle.publish_feedback(feedback)

                goal_handle.abort()
                result = ExecuteReasoning.Result()
                result.success = False
                result.error_message = 'No solutions found'
                return result

        except Exception as e:
            self.get_logger().error(f'Reasoning failed: {e}')
            goal_handle.abort()
            result = ExecuteReasoning.Result()
            result.success = False
            result.error_message = str(e)
            return result

def main(args=None):
    rclpy.init(args=args)
    node = ReasoningActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Techniques

### Dynamic Rule Loading

Load Prolog rules dynamically based on mission context:

```python
class DynamicRuleLoader(Node):
    def __init__(self):
        super().__init__('dynamic_rule_loader')
        self.prolog = Prolog()

        # Base ontology
        self.prolog.consult('/path/to/base_ontology.pl')

    def load_mission_rules(self, mission_type):
        """
        Load mission-specific rules.

        Args:
            mission_type: str, e.g., 'search_and_rescue', 'surveillance'
        """
        rule_files = {
            'search_and_rescue': '/path/to/sar_rules.pl',
            'surveillance': '/path/to/surveillance_rules.pl',
            'inspection': '/path/to/inspection_rules.pl'
        }

        if mission_type in rule_files:
            self.prolog.consult(rule_files[mission_type])
            self.get_logger().info(f'Loaded rules for: {mission_type}')
        else:
            self.get_logger().warn(f'Unknown mission type: {mission_type}')
```

### Prolog Result Parsing

Parse complex Prolog results into ROS 2 messages:

```python
def parse_prolog_results(self, results):
    """
    Parse Prolog query results into structured data.

    Args:
        results: List of Prolog query result dictionaries

    Returns:
        List of parsed objects
    """
    parsed = []

    for result in results:
        # Extract variables
        obj_id = result.get('ID', None)
        obj_class = result.get('Class', None)
        location = result.get('Location', None)

        # Convert to ROS 2 message
        obj_msg = PerceptionObject()
        obj_msg.track_id = int(obj_id.replace('obj_', ''))
        obj_msg.class_name = str(obj_class)

        # Parse location list
        if location:
            coords = eval(location)  # [x, y]
            obj_msg.world_position.x = coords[0]
            obj_msg.world_position.y = coords[1]

        parsed.append(obj_msg)

    return parsed
```

### Multi-Threaded Query Execution

Execute queries in separate thread to avoid blocking ROS 2 callbacks:

```python
import threading
from concurrent.futures import ThreadPoolExecutor

class ThreadedPrologNode(Node):
    def __init__(self):
        super().__init__('threaded_prolog_node')
        self.prolog = Prolog()
        self.executor = ThreadPoolExecutor(max_workers=4)

    def async_query(self, query_string, callback):
        """
        Execute query asynchronously.

        Args:
            query_string: Prolog query
            callback: Function to call with results
        """
        future = self.executor.submit(self.execute_query, query_string)
        future.add_done_callback(
            lambda f: callback(f.result())
        )

    def execute_query(self, query_string):
        """Execute query in separate thread."""
        try:
            results = list(self.prolog.query(query_string))
            return results
        except Exception as e:
            self.get_logger().error(f'Query failed: {e}')
            return []
```

## Performance Optimization

### 1. Batch Assertions

Group related facts to reduce overhead:

```python
def batch_assert_objects(self, objects):
    """Assert multiple objects in batch."""
    # Build multi-line fact string
    facts = []
    for obj in objects:
        fact = (
            f"detected_object("
            f"obj_{obj.track_id}, "
            f"{obj.class_name}, "
            f"[{obj.bbox.x_min}, {obj.bbox.y_min}], "
            f"{obj.confidence}, "
            f"{obj.timestamp}"
            f")"
        )
        facts.append(fact)

    # Assert all facts
    for fact in facts:
        self.prolog.assertz(fact)
```

### 2. Query Caching

Cache frequently-used query results:

```python
from functools import lru_cache

class CachedPrologNode(Node):
    def __init__(self):
        super().__init__('cached_prolog_node')
        self.prolog = Prolog()
        self.query_cache = {}

    def cached_query(self, query_string, ttl=5.0):
        """
        Execute query with caching.

        Args:
            query_string: Prolog query
            ttl: Cache time-to-live in seconds

        Returns:
            Query results
        """
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Check cache
        if query_string in self.query_cache:
            cached_result, timestamp = self.query_cache[query_string]
            if current_time - timestamp < ttl:
                return cached_result

        # Execute query
        results = list(self.prolog.query(query_string))

        # Update cache
        self.query_cache[query_string] = (results, current_time)

        return results
```

### 3. Selective Fact Retention

Only retain facts relevant to current mission:

```python
def cleanup_irrelevant_facts(self, relevant_classes):
    """
    Remove facts not relevant to current mission.

    Args:
        relevant_classes: List of object classes to keep
    """
    # Query all objects
    all_objects = list(self.prolog.query("detected_object(ID, Class, _, _, _)"))

    # Retract irrelevant objects
    for obj in all_objects:
        if obj['Class'] not in relevant_classes:
            self.prolog.retractall(f"detected_object({obj['ID']}, _, _, _, _)")
            self.get_logger().debug(f"Removed irrelevant object: {obj['ID']}")
```

## Error Handling

### Robust Query Execution

```python
def safe_query(self, query_string, max_retries=3):
    """
    Execute query with error handling and retries.

    Args:
        query_string: Prolog query
        max_retries: Maximum retry attempts

    Returns:
        Query results or empty list on failure
    """
    for attempt in range(max_retries):
        try:
            results = list(self.prolog.query(query_string))
            return results
        except Exception as e:
            self.get_logger().warn(
                f'Query attempt {attempt + 1} failed: {e}'
            )
            if attempt == max_retries - 1:
                self.get_logger().error(f'Query failed after {max_retries} attempts')
                return []
```

### Knowledge Base Validation

```python
def validate_knowledge_base(self):
    """
    Validate knowledge base consistency.

    Returns:
        bool: True if valid, False otherwise
    """
    try:
        # Check for required predicates
        required_predicates = [
            'detected_object/5',
            'spatial_relation/3',
            'mission_goal/2'
        ]

        for predicate in required_predicates:
            # Check if predicate exists
            check_query = f"current_predicate({predicate})"
            results = list(self.prolog.query(check_query))

            if not results:
                self.get_logger().error(f'Missing predicate: {predicate}')
                return False

        self.get_logger().info('Knowledge base validation passed')
        return True

    except Exception as e:
        self.get_logger().error(f'Validation failed: {e}')
        return False
```

## Testing

### Unit Testing PySwip Integration

```python
import unittest
from pyswip import Prolog

class TestPySiwpIntegration(unittest.TestCase):
    def setUp(self):
        """Initialize Prolog engine for each test."""
        self.prolog = Prolog()

    def test_assert_and_query(self):
        """Test basic assert and query."""
        self.prolog.assertz("test_fact(a)")
        self.prolog.assertz("test_fact(b)")

        results = list(self.prolog.query("test_fact(X)"))
        self.assertEqual(len(results), 2)
        self.assertIn({'X': 'a'}, results)
        self.assertIn({'X': 'b'}, results)

    def test_retract(self):
        """Test fact retraction."""
        self.prolog.assertz("test_fact(x)")
        self.prolog.retract("test_fact(x)")

        results = list(self.prolog.query("test_fact(x)"))
        self.assertEqual(len(results), 0)

    def test_consult(self):
        """Test loading Prolog file."""
        # Create temporary Prolog file
        with open('/tmp/test.pl', 'w') as f:
            f.write("parent(john, bob).\n")
            f.write("parent(john, alice).\n")

        self.prolog.consult('/tmp/test.pl')
        results = list(self.prolog.query("parent(john, X)"))
        self.assertEqual(len(results), 2)
```

## Common Pitfalls

1. **String Quoting**: Prolog atoms must be properly quoted
   ```python
   # Correct
   prolog.assertz("object('obj_1', person)")

   # Incorrect (will fail if obj_1 starts with capital)
   prolog.assertz("object(obj_1, person)")
   ```

2. **List Syntax**: Prolog lists use different syntax than Python
   ```python
   # Correct
   prolog.assertz("location(obj_1, [10, 20, 30])")

   # Incorrect
   prolog.assertz(f"location(obj_1, {[10, 20, 30]})")  # Will fail
   ```

3. **Memory Management**: Always retract old facts to prevent memory leaks
   ```python
   # Good practice
   prolog.retractall("detected_object(_, _, _, _, _)")
   for obj in new_objects:
       prolog.assertz(f"detected_object(...)")
   ```

4. **Thread Safety**: PySwip is NOT thread-safe
   ```python
   # Use separate Prolog instances for separate threads
   # Or use locks
   import threading
   prolog_lock = threading.Lock()

   with prolog_lock:
       results = list(prolog.query("..."))
   ```

## References

- PySwip GitHub: `/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/pyswip/pyswip/`
- SWI-Prolog Manual: https://www.swi-prolog.org/pldoc/doc_for?object=manual
- ROS 2 Python Client Library: `/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/client_libs/rclpy_api.html`
