import rclpy
from rclpy.node import Node
import numpy as np
import gurobipy as gp
from gurobipy import GRB
from std_msgs.msg import Float64MultiArray



class QPSolver(Node):
    def __init__(self):
        super().__init__('qp_solver')

        self.solution_publisher = self.create_publisher(Float64MultiArray, 'qp_solution', 10)

        self.declare_parameter('Q_flat', [0.0])
        self.declare_parameter('Q_dim', 1)
        self.declare_parameter('A_flat', [0.0])
        self.declare_parameter('A_rows', 1)
        self.declare_parameter('A_cols', 1)
        self.declare_parameter('b', [0.0])
        self.declare_parameter('c', [0.0])

        Q_flat = self.get_parameter('Q_flat').get_parameter_value().double_array_value
        Q_dim = self.get_parameter('Q_dim').get_parameter_value().integer_value
        A_flat = self.get_parameter('A_flat').get_parameter_value().double_array_value
        A_rows = self.get_parameter('A_rows').get_parameter_value().integer_value
        A_cols = self.get_parameter('A_cols').get_parameter_value().integer_value
        b_vec = self.get_parameter('b').get_parameter_value().double_array_value
        c_vec = self.get_parameter('c').get_parameter_value().double_array_value

        try:
            Q = np.array(Q_flat).reshape((Q_dim, Q_dim))
            A = np.array(A_flat).reshape((A_rows, A_cols))
            b = np.array(b_vec)
            c = np.array(c_vec)

            self.get_logger().info(f"✅ Q matrix:\n{Q}")
            self.get_logger().info(f"✅ A matrix:\n{A}")
            self.get_logger().info(f"✅ b vector:\n{b}")
            self.get_logger().info(f"✅ c vector:\n{c}")

            self.get_logger().info("🔍 Solving the following QP:")
            self.get_logger().info("  minimize (1/2) * x^T Q x + c^T x")
            self.get_logger().info("  subject to A x >= b")

        except ValueError as e:
            self.get_logger().error(f"❌ Parameter shape mismatch: {e}")
            return 

        # GUROBI based solver
        try:
            model = gp.Model("ros2_qp")
            model.setParam('OutputFlag', 0)  

            x = model.addMVar(shape=Q_dim, name="x")

            objective = 0.5 * x @ Q @ x + c @ x
            model.setObjective(objective, GRB.MINIMIZE) # if obj is to min
            # model.setObjective(obj, GRB.MAXIMIZE) if objective is to maximize


            model.addConstr(A @ x >= b, name="geq")
            #model.addConstr(A @ x <= b, name="leq") # if the inequality constraints "<=" is included
            #model.addConstr(A @ x == b, name="eq")  #equality constraints

            model.optimize()

            if model.status == GRB.OPTIMAL:
                self.get_logger().info(f"🌟 Optimal solution x* = {x.X}")
                self.get_logger().info(f"💰 Optimal cost = {model.ObjVal}")
                msg = Float64MultiArray()
                msg.data = x.X.tolist()
                self.solution_publisher.publish(msg)
                self.get_logger().info("📤 Published optimal solution to /qp_solution")

            else:
                self.get_logger().warn(f"⚠️ Solver returned status: {model.status}")

        except Exception as e:
            self.get_logger().error(f"❌ Gurobi solver exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = QPSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
