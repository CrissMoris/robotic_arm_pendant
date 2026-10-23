#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

def send_simple_positions():
    rospy.init_node('orion5_simple_positions')
    
    # Controller publisher
    pub = rospy.Publisher('/orion_arm_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    
    rospy.sleep(2)  # Clock başlaması için bekle
    
    def create_trajectory(positions, duration, description=""):
        """Tek pozisyon için trajectory oluştur"""
        traj = JointTrajectory()
        traj.joint_names = [
            'joint_1',
            'joint_2', 
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]
        
        # Tek nokta trajectory
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        
        traj.points = [point]
        
        rospy.loginfo(f"🎯 {description} gönderiliyor...")
        rospy.loginfo(f"Joint değerleri: {positions}")
        pub.publish(traj)
        rospy.loginfo("✅ Gönderildi!")
        
    # Pozisyon tanımları
    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    down_looking_position = [
        0.0,                    # joint1 = 0°
        math.radians(10),       # joint2 = 10°
        math.radians(80),       # joint3 = 80°
        math.radians(-90),      # joint4 = -90°
        0.0,                    # joint5 = 0°
        0.0                     # joint6 = 0°
    ]
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("ORION5 BASİT POZİSYON KONTROLÜ")
    rospy.loginfo("=" * 50)
    rospy.loginfo("Komutlar:")
    rospy.loginfo("1 - Home pozisyonu (0,0,0,0,0,0)")
    rospy.loginfo("2 - Aşağı bakan pozisyon (0°,0°,90°,-90°,0°,0°)")
    rospy.loginfo("q - Çıkış")
    
    while not rospy.is_shutdown():
        try:
            command = input("\nKomut girin: ").strip().lower()
            
            if command == '1':
                create_trajectory(home_position, 3.0, "Home Pozisyonu")
                
            elif command == '2':
                create_trajectory(down_looking_position, 4.0, "Aşağı Bakan Pozisyon")
                
            elif command == 'q':
                rospy.loginfo("Program sonlandırılıyor...")
                break
                
            else:
                rospy.loginfo("❌ Sadece 1, 2 veya q!")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            rospy.logerr(f"Hata: {str(e)}")

if __name__ == '__main__':
    try:
        send_simple_positions()
    except rospy.ROSInterruptException:
        pass