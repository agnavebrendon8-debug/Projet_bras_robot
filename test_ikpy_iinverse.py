from ikpy.chain import Chain
from ikpy.link import URDFLink

# Définition d'une chaîne simple
from ikpy.link import URDFLink
from ikpy.chain import Chain

robot_arm = Chain(name='arm', links=[

    URDFLink(
        name="base",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1]
    ),

    URDFLink(
        name="joint1",
        origin_translation=[0, 0, 10],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),

    URDFLink(
        name="joint2",
        origin_translation=[10, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    )
])

# Calcul IK pour atteindre une position cible
target = [1.5, 0, 1.0]
angles = robot_arm.inverse_kinematics(target)
print("Angles articulations :", angles)