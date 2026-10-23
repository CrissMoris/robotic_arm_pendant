#!/usr/bin/env python3
"""
square_trajectory.py – Orion5 için kartesyen kare yörüngesi çizer,
başlangıç pozu komut satırı argümanlarıyla belirlenir.
"""

import sys, copy                     
import rospy, moveit_commander, geometry_msgs.msg  
from tf.transformations import quaternion_from_euler  

def get_pose_by_index(i, target, square_size):
    """
    i. köşeye karşılık gelen pozu üretir.
    Kare biçiminde, target pozundan başlayarak ilerler.
    """
    pose = copy.deepcopy(target)  # Hedef pozu bozmadan kopyalamanızı sağlar
    if i == 0:
        pass  # Başlangıç noktası (hiçbir şey değiştirme)
    elif i == 1:
        pose.position.x += square_size  # Sağ köşe: X ekseninde ileri
    elif i == 2:
        pose.position.x += square_size
        pose.position.y += square_size  # Sağ üst köşe: X ve Y ileri
    elif i == 3:
        pose.position.y += square_size  # Sol üst köşe: Y ileri
    elif i == 4:
        pass  # Kapanış/başa dönüş (başlangıç noktası)
    return pose

def main():
    # Komut satırından doğru sayıda argüman alındı mı kontrolü
    if len(sys.argv) == 9:
        x, y, z = map(float, sys.argv[1:4])            # Pozisyon (x, y, z)
        qx, qy, qz, qw = map(float, sys.argv[4:8])     # Quaternion oryantasyonu
        duration = float(sys.argv[8]) + 5.0            # Hareket süresi + 5 saniye ek gecikme
    else:
        print("Usage:\n  rosrun orion5_interface square_trajectory.py x y z qx qy qz qw duration")
        return

    # ROS ve MoveIt başlatma adımları
    moveit_commander.roscpp_initialize(sys.argv)       # MoveIt! arayüzünü başlatır
    rospy.init_node("square_traj_script", anonymous=True)  # Yeni ROS node başlatılır

    # Hareket grubu tanımlama (robot kolu)
    group = moveit_commander.MoveGroupCommander("orion5_arm")
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)

    rospy.sleep(0.5)                                   # Sistemlerin inisiyalize olması için kısa bekleme
    group.set_pose_reference_frame("base_link")        # Referans çerçevesini taban (base_link) olarak ayarlar
    group.set_planning_time(10.0)                      # Planlama için azami süreyi 10s yapar

    # Başlangıç pozu (z + 0.1 m) için hedef poz oluşturuluyor
    initial_pose = geometry_msgs.msg.Pose()
    initial_pose.position.x, initial_pose.position.y, initial_pose.position.z = x, y, z + 0.3
    initial_pose.orientation.x = qx
    initial_pose.orientation.y = qy
    initial_pose.orientation.z = qz
    initial_pose.orientation.w = qw

    # Başlangıç pozuna gitmek için hareket planlanır ve uygulanır
    try:
        group.set_pose_target(initial_pose)                # Hedef pozu ayarla
        rospy.loginfo("Planning to initial pose: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)", 
                      initial_pose.position.x, initial_pose.position.y, initial_pose.position.z,
                      initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w)
        success = group.go(wait=True)                      # Pozisyona git ve bekle
        if not success:
            rospy.logerr("Failed to move to initial pose (IK may be unsolvable)")
            moveit_commander.roscpp_shutdown()
            return
        group.stop()                                       # Hareketi tamamen durdur
        group.clear_pose_targets()                         # Hedefleri sıfırla
        rospy.sleep(0.1)                                   # Kısa bekleme
    except Exception as e:
        rospy.logerr(f"Failed to plan/execute initial pose motion: {e}")
        moveit_commander.roscpp_shutdown()
        return

    # Komut satırı argümanlarından hedef poz (Pose) oluşturuluyor
    target = geometry_msgs.msg.Pose()
    target.position.x, target.position.y, target.position.z = x, y, z
    target.orientation.x = qx
    target.orientation.y = qy
    target.orientation.z = qz
    target.orientation.w = qw

    square_size = 0.2  # Karenin bir kenarının uzunluğu (metre cinsinden: 20cm)

    num_corners = 5    # Kaç köşe gezilecek? Kare için 4 köşe, 5. adım kapanış/başa dönüş.
    # Her köşe arasında harcanacak zamanı hesapla (toplam süreye eşit yayılır)
    single_step_duration = duration / (num_corners-1) if num_corners > 1 else duration

    # Her köşe için ayrı ayrı hareket planlanır ve uygulanır
    for i in range(num_corners):
        pose_i = get_pose_by_index(i, target, square_size)  # i. köşeyi bul
        group.set_start_state_to_current_state()            # Başlangıç durumunu güncelle
        plan, fraction = group.compute_cartesian_path([pose_i], 0.01, True)  # Yolu planla
        if fraction < 0.5:                                 # Yolun tamamı üretilemediyse uyar ve çık
            rospy.logwarn("Köşe %d kartesyen yol tamamlanamadı (%.1f %%)", i, fraction * 100)
            moveit_commander.roscpp_shutdown()
            return
        
        # Planlanmış yolun süresini hedef süreyle ölçeklendir (zaman yeniden dağıtılıyor)
        plan_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
        s = single_step_duration / plan_time if plan_time > 0.0 and single_step_duration != plan_time else 1.0
        for pt in plan.joint_trajectory.points:
            pt.time_from_start = rospy.Duration.from_sec(pt.time_from_start.to_sec() * s)
            if pt.velocities:
                pt.velocities = [v / s for v in pt.velocities]
            if pt.accelerations:
                pt.accelerations = [a / s**2 for a in pt.accelerations]

                

        group.execute(plan, wait=True)                     # Planı çalıştır, hareketi bekle




    joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Ön tanımlı bir eklem açı dizisi (radyan cinsinden)
    try:
        group.set_joint_value_target(joint_angles)         # Hedef eklem açılarını ayarla
        rospy.loginfo("Planning to joint position: %s", joint_angles)
        success = group.go(wait=True)                      # Pozisyona git ve bekle
        if not success:
            rospy.logerr("Failed to move to joint position")
            moveit_commander.roscpp_shutdown()
            return
        group.stop()                                       # Hareketi tamamen durdur
        group.clear_pose_targets()                         # Hedefleri sıfırla
        rospy.sleep(0.1)                                   # Kısa bekleme
    except Exception as e:
        rospy.logerr(f"Failed to plan/execute joint motion: {e}")
        moveit_commander.roscpp_shutdown()
        return

    moveit_commander.roscpp_shutdown()                     # ROS ve MoveIt'i kapat

if __name__ == "__main__":
    main()
