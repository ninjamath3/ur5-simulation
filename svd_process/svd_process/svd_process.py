import rclpy
from geometry_msgs.msg import Point  # Importation du type de message Point
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import time

# Liste globale pour stocker les données reçues
global points_camera
global points_3D
points_camera = []
points_3D = []
# Flag pour savoir quand le robot a fini de bouger
global flag_value

# Dictionnaire pour les données articulaires
global articulations
articulations = {}

def c(theta):
    return math.cos(theta)

def s(theta):
    return math.sin(theta)

def get_MG(rx, ry, rz, x, y, z):
    Rx = [[1, 0, 0], [0, c(rx), -s(rx)], [0, s(rx), c(rx)]]
    Rx = np.array(Rx)

    Ry = [[c(ry), 0, s(ry)], [0, 1, 0], [-s(ry), 0, c(ry)]]
    Ry = np.array(Ry)

    Rz = [[c(rz), -s(rz), 0], [s(rz), c(rz), 0], [0, 0, 1]]
    Rz = np.array(Rz)

    R = Rz @ Ry @ Rx

    T = [[x], [y], [z]]
    T = np.array(T)

    MG = np.eye(4)
    MG[:3, :3] = R
    MG[:3, 3] = T.flatten()

    return MG

def compute_svd(points_3d, points_2d):
    """
    Permet de réaliser la décomposition svd
    """
    num_points = points_3d.shape[0]

    A = []
    for i in range(num_points):
        X, Y, Z = points_3d[i]
        u, v, _ = points_2d[i]
        A.append([X, Y, Z, 1, 0, 0, 0, 0, -u*X, -u*Y, -u*Z, -u])
        A.append([0, 0, 0, 0, X, Y, Z, 1, -v*X, -v*Y, -v*Z, -v])

    A = np.array(A)
    U, S, Vt = np.linalg.svd(A)

    M = Vt[-1].reshape(3, 4)

    return M

def point_callback_points_camera(msg):
    # Callback pour traiter les messages reçus.
    print(f'Point reçu caméra : x={msg.x}, y={msg.y}, z={msg.z}')
    points_camera.append([msg.x, msg.y, msg.z])

def point_callback_points_3D(msg):
    # Callback pour traiter les messages reçus.
    print(f'Point 3D reçu : x={msg.x}, y={msg.y}, z={msg.z}')
    points_3D.append([msg.x, msg.y, msg.z])

def flag_callback(msg):
    global flag_value
    flag_value = msg.data

def joint_state_callback(msg):
    global articulations
    joint_names = msg.name
    joint_positions = msg.position
    for name, position in zip(joint_names, joint_positions):
        articulations[name] = position

def main(args=None):
    """Point d'entrée principal du programme."""
    rclpy.init(args=args)

    # --- Pour les points 3D ---
    node_3D = rclpy.create_node('point_listener_3D')  # Crée un nœud pour les points 3D

    # Abonnement au topic
    subscription_mathys_3D = node_3D.create_subscription(
        Point,                       # Type de message
        'point_topic_mathys_3D',     # Nom du topic
        point_callback_points_3D,    # Fonction callback associée
        20                           # Taille de la queue
    )
    subscription_mathys_3D  # Empêche le garbage collector de supprimer la subscription

    # --- Pour les points caméra ---
    node_camera = rclpy.create_node('point_listener_camera')  # Crée un nœud pour les points caméra

    # Abonnement au topic
    subscription_camera = node_camera.create_subscription(
        Point,                           # Type de message
        'apriltag_pixel_coordinates',    # Nom du topic
        point_callback_points_camera,    # Fonction callback
        20                               # Taille de la queue
    )
    subscription_camera  # Empêche le garbage collector de supprimer la subscription

    print('Node prêt à écouter les messages de type Point.')

    #------------Pour le flag-----------------
    node_flag = rclpy.create_node("flag_subscriber")
    node_flag.create_subscription(Bool, "/flag_topic", flag_callback, 10)

    try:
        global points_3D
        rclpy.spin_once(node_flag)
        print(f"valeur de flag_value : {flag_value}")
        while not flag_value:
            print('Ecoute de nouveaux points en cours')
            rclpy.spin_once(node_3D)
            if points_3D[-1][0] == 0 and points_3D[-1][1] == 0 and points_3D[-1][2] == 0:
                points_3D.pop()
            else:
                rclpy.spin_once(node_camera)
                
            rclpy.spin_once(node_flag)
            print(f"valeur du flag : {flag_value}")
            time.sleep(2)
           
    except KeyboardInterrupt:
        print('\nArrêt des nœuds.')

    finally:
        global points_camera
        global articulations
        
        print(f'Points caméra stockés : {points_camera}\n')
        print(f'Points 3D stockés : {points_3D}')
        node_camera.destroy_node()
        node_3D.destroy_node()
        node_flag.destroy_node()

        node_JointState = rclpy.create_node("joint_state_listener")
        node_JointState.create_subscription(JointState, "/joint_states", joint_state_callback, 10)

        try:
            rclpy.spin_once(node_JointState)
        except KeyboardInterrupt:
            print("\nArrêt des nœuds")
        finally:
            node_JointState.destroy_node()

        print(f"Voici les informations des joints obtenues : {articulations}")
        
        #On rentre la matrice des paramètres intrinsèques et on l'inverse
        K_list = [[520.7813804684724, 0.0, 320.5], [0.0, 520.7813804684724, 240.5], [0.0, 0.0, 1.0]]
        K = np.array(K_list)
        
        #On transforme les listes globales en tableau numpy afin d'appliquer svd
        points_3D = np.array(points_3D)
        points_camera = np.array(points_camera)
        
        #On a le résultat de svd
        M = compute_svd(points_3D, points_camera)

        #On inverse K pour n'avoir que la matrice des paramètres intrinsèques
        K_inv = np.linalg.inv(K)
        A = K_inv @ M

        #On isole la rotation et la translation
        R = A[:, :3]
        Rt = -A[:, 3:]

        R_inv = np.linalg.inv(R)
        t = R_inv @ Rt
        
	#On forme la matrice homogène donnant la position de l'effecteur par rapport à la caméra
        Mc6 = np.eye(4)
        Mc6[:3, :3] = R
        Mc6[:3, 3] = t.flatten()

	#On prélève les valeurs des rotations en radian de chacun des joints 
        joints_values = [articulations[key] for key in articulations.keys()]
        q1 = joints_values[0]
        q2 = joints_values[1]
        q3 = joints_values[2]
        q4 = joints_values[3]
        q5 = joints_values[4]
        q6 = joints_values[5]

	#On forme chacune des matrices homogènes grâce au fichier urdf défini 
        pi = math.pi
        M01 = get_MG(0, 0, q1, 0, 0, 0.1625)
        M12 = get_MG(pi / 2.0, q2, 0, 0, 0, 0)
        M23 = get_MG(0, 0, q3, -0.425, 0, 0)
        M34 = get_MG(0, 0, q4, -0.3922, 0, 0.1333)
        M45 = get_MG(pi / 2.0, q5, 0, 0, -0.0997, 0)
        M56 = get_MG(pi / 2.0, pi, pi + q6, 0, 0.0996, 0)
        
        
	#On multiplie toutes les matrices homogènes entre elles afin d'obtenir la position de l'effecteur par rapport à la base 
        M06 = M01 @ M12 @ M23 @ M34 @ M45 @ M56
        
	#On multiplie Mc6 avec l'inverse de M06 afin d'obtenir la position de la base du robot par rapport à la caméra       	        
        Mc0 = Mc6 @ np.linalg.inv(M06)

	print("---------------------------------------------------------------------")
        print("\n Voici la matrice homogène entre la caméra et la base du robot : \n", Mc0)
	print("---------------------------------------------------------------------")

        rclpy.shutdown()

if __name__ == '__main__':
    main()

