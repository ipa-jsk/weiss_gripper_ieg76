#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from weiss_gripper_ieg76.srv import Move, MoveResponse, SetForce, SetForceResponse

from weiss_gripper_ieg76.state_publisher import StatesPublisher
from sensor_msgs.msg import JointState
from driver import Driver
from threading import Thread, Lock


class Simulator(Driver):
	def __init__(self):

		rospy.on_shutdown(self.shutdown_handler)
		self.gripper_pose = 0
		self.velocity = rospy.get_param("~velocity",100) # mm/s
		self.rate = 100
		self.activated = True
		self.pose_lock = Lock()

		self.joint_state_msg = JointState()
		self.joint_state_msg.name = []
		self.joint_state_msg.name.append("ur5_weiss_ieg76_jaw_position")  # TODO use parameter
		self.joint_states_publisher = rospy.Publisher('~joint_states', JointState, queue_size=10)
		self.states_publisher_thread = Thread(target = self.publish_states)

	def publish_states(self):
		r = rospy.Rate(self.rate)
		while(not rospy.is_shutdown()):
			self.joint_state_msg.header.stamp = rospy.Time.now()
			self.joint_state_msg.position = []
			MM_TO_M = 0.001
			self.pose_lock.acquire()
			joint_state = self.gripper_pose * MM_TO_M / 2.0  # joint state is half the stroke
			self.pose_lock.release()
			self.joint_state_msg.position.append(joint_state)
			try:
				self.joint_states_publisher.publish(self.joint_state_msg)
			except:
				rospy.logerr("\nClosed topics.")
			r.sleep()

	def move_to(self, pose):
		r = rospy.Rate(self.rate)
		diff = pose-self.gripper_pose
		while (abs(diff) > 0.001):
			diff = pose-self.gripper_pose
			if abs(diff) > self.velocity / self.rate:
				self.pose_lock.acquire()
				self.gripper_pose = self.gripper_pose + (diff / abs(diff)) * self.velocity / self.rate
				self.pose_lock.release()
			else: # done
				self.pose_lock.acquire()
				self.gripper_pose = pose
				self.pose_lock.release()
				return
			r.sleep()


	def set_max_pos(self, product_id):
		if product_id == "CRG 200-085":
			self._max_pos = 85
		elif product_id == "CRG 30-050":
			self._max_pos = 50
		elif product_id == "IEG 76-030":
			self._max_pos = 30
		elif product_id == "IEG 55-020":
			self._max_pos = 20
		else:
			raise RuntimeError("Gripper type '{}' not yet supported!".format(product_id))

	def handle_reference(self, req):
		rospy.loginfo("Referencing")
		self.move_to(self._max_pos)
		return TriggerResponse()

	def relative_to_absolute(self, req):
		if req.relative:
			req.position = self.gripper_pos + req.position

	def handle_open(self, req):
		rospy.loginfo("Opening")
		reply = MoveResponse(success=True)
		self.relative_to_absolute(req)
		if not self.check_position(req.position):
			reply.success = False
			reply.message = 'Opening failed. Position must be 0.0(mm) <= position <= {}.0(mm).'.format(self._max_pos)
		else:
			self.move_to(req.position)
			reply.message = 'Opening successful'
		self.log_reply(reply)
		return reply

	def handle_close(self, req):
		rospy.loginfo("Closing")
		reply = MoveResponse(success=True)
		self.relative_to_absolute(req)
		if not self.check_position(req.position):
			reply.success = False
			reply.message = 'Closing failed. Position must be 0.0(mm) <= position <= {}.0(mm).'.format(self._max_pos)
		else:
			self.move_to(req.position)
			reply.message = 'Closing successful'
		self.log_reply(reply)
		return reply

	def handle_grasp(self, req):
		rospy.loginfo("Grasping")
		reply = MoveResponse(success=True)
		self.relative_to_absolute(req)
		if not self.check_position(req.position):
			reply.success = False
			reply.message = 'Grasping failed. Position must be 0.0(mm) <= position <= {}.0(mm).'.format(self._max_pos)
		else:
			self.move_to(req.position)
			reply.message = 'Grasping successful'
		self.log_reply(reply)
		return reply

	def handle_set_force(self, req):
		rospy.loginfo("Set force")
		reply = SetForceResponse(success=True)
		if not self.check_force(req.grasping_force):
			reply.success = False
			reply.message = 'Force must be 0(%) <= force <= 100(%).'
		else:
			rospy.sleep(0.2) # simluated set force time
			reply.success = True
		if reply.success:
			reply.message = 'Set force successful.'
		else:
			reply.message = 'Set force failed. ' + reply.message
		self.log_reply(reply)
		return reply

	def handle_ack(self, req):
		rospy.loginfo("Ack error")
		reply = TriggerResponse(success=True)
		self.activated = True
		rospy.sleep(0.2) # simluated handle ack time
		reply.message = 'Ack error ' + reply.message
		self.log_reply(reply)
		return reply

	def handle_deactivate(self, req):
		rospy.loginfo("Deactivate gripper")
		reply = TriggerResponse(success=True)
		self.activated = False
		rospy.sleep(0.1) # simulated deactivation time
		reply.message = 'Deactivating ' + reply.message
		self.log_reply(reply)
		return reply

	def shutdown_handler(self):
		# self.states_publisher_thread.shutdown()
		rospy.loginfo("Gracefully shutting down the driver...")

	def run(self):

		rospy.logdebug("Starting threads...")
		self.states_publisher_thread.start()
		rospy.logdebug("Threads started.")

		rospy.loginfo('Receiving info about gripper')

		# update parameters based on product id
		product_id = rospy.get_param("~product_id", None)
		self.set_max_pos(product_id)

		grasp_force = rospy.get_param("~grasping_force", 100)

		serv_ref = rospy.Service('~reference', Trigger, self.handle_reference)
		serv_ref = rospy.Service('~open', Move, self.handle_open)
		serv_ref = rospy.Service('~close', Move, self.handle_close)
		serv_ref = rospy.Service('~grasp', Move, self.handle_grasp)
		serv_ref = rospy.Service('~ack', Trigger, self.handle_ack)
		serv_ref = rospy.Service('~set_force', SetForce, self.handle_set_force)
		serv_ref = rospy.Service('~deactivate', Trigger, self.handle_deactivate)

		rospy.loginfo("Ready to receive requests.")

		rospy.spin()


if __name__ == "__main__":
	# rospy.init_node('ieg_driver', log_level=rospy.DEBUG)
	rospy.init_node('ieg_driver')

	driver = Simulator()
	driver.run()
