import numpy as np
import matplotlib.pyplot as plt

# Link lengths
l1, l2 = 5, 3

# ---------------- Utility Functions ----------------

def forward_kinematics(theta1_deg, theta2_deg):
    """Calculate positions of joints given angles"""
    theta1 = np.radians(theta1_deg)
    theta2 = np.radians(theta2_deg)

    P0 = np.array([0, 0])
    P1 = np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    P2 = P1 + np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])

    return P0, P1, P2

def inverse_kinematics(x, y):
    """Calculate joint angles for target point (x,y)"""
    d = np.sqrt(x**2 + y**2)
    if d > (l1 + l2) or d < abs(l1 - l2):
        print("⚠️ Target point is out of reach!")
        return None, None

    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2*l1*l2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))
    theta1 = np.arctan2(y, x) - np.arctan2(l2*np.sin(theta2), l1 + l2*np.cos(theta2))

    return np.degrees(theta1), np.degrees(theta2)

def angle(a, b, c):
    """Angle at point b given 3 points a,b,c"""
    ba = a - b
    bc = c - b
    cos_theta = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    return np.degrees(np.arccos(cos_theta))

def plot_arm(P0, P1, P2, title="Robotic Arm"):
    """Plot the robotic arm as a triangle"""
    plt.figure()
    plt.plot([P0[0], P1[0], P2[0], P0[0]], [P0[1], P1[1], P2[1], P0[1]], 'bo-')
    plt.text(P0[0], P0[1], ' P0 (Origin)')
    plt.text(P1[0], P1[1], ' P1 (Joint1)')
    plt.text(P2[0], P2[1], ' P2 (End Effector)')
    plt.axis("equal")
    plt.grid(True)
    plt.title(title)
    plt.show()

# ---------------- Main Program Loop ----------------

while True:
    print("\nRobotic Arm Simulation")
    print("1. Forward Kinematics (Input Angles)")
    print("2. Inverse Kinematics (Input Target Point)")
    print("3. Exit")
    
    choice = input("Choose option (1, 2 or 3): ")

    if choice == "1":
        theta1_deg = float(input("Enter angle θ1 (in degrees): "))
        theta2_deg = float(input("Enter angle θ2 (in degrees): "))

        P0, P1, P2 = forward_kinematics(theta1_deg, theta2_deg)

        print("\nJoint Positions:")
        print("P0 (Origin):", P0)
        print("P1 (Joint1):", P1)
        print("P2 (End Effector):", P2)

        angle_P0 = angle(P1, P0, P2)
        angle_P1 = angle(P0, P1, P2)
        angle_P2 = angle(P0, P2, P1)
        print(f"Triangle Angles: P0={angle_P0:.2f}°, P1={angle_P1:.2f}°, P2={angle_P2:.2f}°")
        print(f"Check Sum ≈ 180°: {angle_P0+angle_P1+angle_P2:.2f}°")

        plot_arm(P0, P1, P2, "Forward Kinematics")

    elif choice == "2":
        x = float(input("Enter target X coordinate: "))
        y = float(input("Enter target Y coordinate: "))

        theta1, theta2 = inverse_kinematics(x, y)

        if theta1 is not None:
            print(f"Calculated Angles: θ1={theta1:.2f}°, θ2={theta2:.2f}°")
            P0, P1, P2 = forward_kinematics(theta1, theta2)
            plot_arm(P0, P1, P2, "Inverse Kinematics Result")

    elif choice == "3":
        print("Exiting simulation... Goodbye 👋")
        break

    else:
        print("⚠️ Invalid choice! Try again.")
