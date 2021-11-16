
    def truck_state_handler_new(self, truck_state):
        """Transform frames instead of markers"""
        who = truck_state.id

        if who not in self.truck_markers:
            print "Received update for truck with unknown id %d", who
        model = self.truck_markers[who]

        cab_x, cab_y, trailer_middle_x, trailer_middle_y = self.calculate_positions(truck_state, model)

        # Generate cab transform
        cab_frame = model.cab.header.frame_id
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = cab_frame
        t.transform.translation.x = cab_x
        t.transform.translation.y = self.map_height - cab_y
        t.transform.translation.z = 0

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, -truck_state.theta1)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.truck_pub.publish(model.cab)
        self.truck_pub.publish(model.label)
        tf2_ros.TransformBroadcaster().sendTransform(t)

        # Generate trailer transform
        trailer_frame = model.trailer.header.frame_id
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = trailer_frame
        t.transform.translation.x = trailer_middle_x
        t.transform.translation.y = self.map_height - trailer_middle_y
        t.transform.translation.z = 0

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, -truck_state.theta2)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.truck_pub.publish(model.trailer)
        tf2_ros.TransformBroadcaster().sendTransform(t)