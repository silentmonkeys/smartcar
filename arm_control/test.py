from running import ActionStep, get_arm


arm = get_arm()

# 单关节控制 Single-servo control
arm.set(1, 90, duration=0.3)

# 六关节控制 Six-servo control
arm.set_all(90, 90, 90, 90, 90, 180, duration=0.5)

# 动作编排 Action sequence
wave = [
    ActionStep({2: 70, 3: 130}, 0.3),
    ActionStep({2: 120, 3: 90}, 0.3),
]
arm.add_action("wave", wave)
arm.action("wave", loop=2)

arm.close()
del arm
