// Optimize the IK calculations for Piper arms using WebXR VR controllers.
// World vectors after rotation: -X is forward, -Y is left/right (Yaw), Z is up/down.
// 

linkLengths: {
    joint2: 0.285,  // Upper arm (link2)
    joint4: 0.2518, // Forearm (link3 to link4)
    joint6: 0.091,  // Wrist to gripper base
    joint7: 0.1358, // Gripper base to finger (first finger)
    joint8: 0.1358  // Gripper base to finger (second finger)
},
jointControls: [
    { jointName: 'left_joint1', lowerLimit: -2.618, upperLimit: 2.618, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'left_joint2', lowerLimit: 0, upperLimit: 3.14, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'left_joint3', lowerLimit: -2.967, upperLimit: 0, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'left_joint4', lowerLimit: -1.745, upperLimit: 1.745, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'left_joint5', lowerLimit: -1.22, upperLimit: 1.22, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'left_joint6', lowerLimit: -2.0944, upperLimit: 2.0944, axis: [0, 0, -1], isPrismatic: false },
    { jointName: 'left_joint7', lowerLimit: -0.035, upperLimit: 0, axis: [0, 1, 0], isGripper: true, isPrismatic: true },
    { jointName: 'left_joint8', lowerLimit: -0.035, upperLimit: 0, axis: [0, -1, 0], isGripper: true, isPrismatic: true },
    { jointName: 'right_joint1', lowerLimit: -2.618, upperLimit: 2.618, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'right_joint2', lowerLimit: 0, upperLimit: 3.14, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'right_joint3', lowerLimit: -2.967, upperLimit: 0, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'right_joint4', lowerLimit: -1.745, upperLimit: 1.745, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'right_joint5', lowerLimit: -1.22, upperLimit: 1.22, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'right_joint6', lowerLimit: -2.0944, upperLimit: 2.0944, axis: [0, 0, 1], isPrismatic: false },
    { jointName: 'right_joint7', lowerLimit: -0.035, upperLimit: 0, axis: [0, 1, 0], isGripper: true, isPrismatic: true },
    { jointName: 'right_joint8', lowerLimit: -0.035, upperLimit: 0, axis: [0, -1, 0], isGripper: true, isPrismatic: true }
]

computeArmIK(hand, targetPos, deltaTime, targetOrientation = null) {
        const prefix = hand === 'left' ? 'left' : 'right';
        const linkLengths = {
            upperArm: this.config.linkLengths.joint2 * this.config.scaleFactor,
            forearm: this.config.linkLengths.joint4 * this.config.scaleFactor,
            wristToGripper: this.config.linkLengths.joint6 * this.config.scaleFactor
        };
        const maxReach = linkLengths.upperArm + linkLengths.forearm + linkLengths.wristToGripper;

        // Initialize storage for recent valid angles and target.x
        this.recentJointAngles = this.recentJointAngles || {
            left: { joint1: 0, joint2: 0, joint3: 0, joint4: 0, joint5: 0, joint6: 0 },
            right: { joint1: 0, joint2: 0, joint3: 0, joint4: 0, joint5: 0, joint6: 0 }
        };
        
        this.recentTargetX = this.recentTargetX || {
            left: 0,
            right: 0
        };

        // Step 1: Validate input
        if (!targetPos || !isFinite(targetPos.x + targetPos.y + targetPos.z)) {
            console.warn(`${prefix} computeArmIK: Invalid target position, skipping`);
            return Array(6).fill(0);
        }

        // Step 2: Transform target position to robot base frame
        const basePos = this.initialRobotPosition.clone();
        const baseRotation = new Matrix4().makeRotationFromEuler(new Euler(-Math.PI / 2, 0, Math.PI / 2, 'XYZ'));
        let target = targetPos.clone().sub(basePos).applyMatrix4(baseRotation.invert());
        if (this.debug) {
            console.log(`${prefix} target in base frame: x=${target.x.toFixed(3)}, y=${target.y.toFixed(3)}, z=${target.z.toFixed(3)}`);
        }

        // Step 3: Clamp target to reachable distance
        const targetDist = target.length();
        let scale = 1;
        if (targetDist > maxReach) {
            scale = maxReach / targetDist;
            target.multiplyScalar(scale);
            if (this.debug) {
                console.log(`${prefix} target clamped to max reach: ${maxReach.toFixed(3)}, scale=${scale.toFixed(3)}`);
            }
        }

        // Step 4: Define joint names and controls
        const armJoints = [
            `${prefix}_joint1`, // Yaw
            `${prefix}_joint2`, // Pitch
            `${prefix}_joint3`, // Elbow
            `${prefix}_joint4`, // Wrist1
            `${prefix}_joint5`, // Wrist2
            `${prefix}_joint6`  // Wrist3
        ];
        const jointControls = this.config.jointControls.filter(c => c.jointName.startsWith(`${prefix}_`) && !c.isGripper);
        const jointUpdates = {};

        // Step 5: Define joint-specific maxDelta for smoothing
        const jointMaxDelta = {
            [`${prefix}_joint1`]: 3.5 * deltaTime, // Yaw
            [`${prefix}_joint2`]: 5.0 * deltaTime, // Pitch
            [`${prefix}_joint3`]: 5.5 * deltaTime, // Elbow
            [`${prefix}_joint4`]: 4.0 * deltaTime, // Wrist1
            [`${prefix}_joint5`]: 4.0 * deltaTime, // Wrist2
            [`${prefix}_joint6`]: 4.0 * deltaTime  // Wrist3
        };

        // Step 6: Compute yaw (joint1)
        const joint1Control = jointControls.find(c => c.jointName === armJoints[0]);
        const xzDist = Math.sqrt(target.x ** 2 + target.z ** 2 + 1e-10);
        let yaw;
        const yawOffset = 0;
        if (hand === 'left') {
            yaw = Math.atan2(target.y, xzDist);
            yaw = -yaw - .2;
        } else {
            yaw = Math.atan2(target.y, xzDist);
            yaw = yaw + .2;
        }
        if (yaw < joint1Control.lowerLimit || yaw > joint1Control.upperLimit) {
            jointUpdates[armJoints[0]] = this.recentJointAngles[hand].joint1;
        } else {
            jointUpdates[armJoints[0]] = yaw;
            this.recentJointAngles[hand].joint1 = yaw;
        }
        if (this.debug) {
            console.log(`${prefix}_joint1 (yaw with offset ${yawOffset}): ${jointUpdates[armJoints[0]].toFixed(3)} rad, xzDist=${xzDist.toFixed(3)}`);
        }

        // Step 7: Compute pitch (joint2)
        const joint2Control = jointControls.find(c => c.jointName === armJoints[1]);
        let pitch;
        let pitchOffset = 0;
        pitch = Math.atan2(-target.z, -target.x) + pitchOffset;

        if (pitch < joint2Control.lowerLimit || pitch > joint2Control.upperLimit) {
            jointUpdates[armJoints[1]] = this.recentJointAngles[hand].joint2;
        } else {
            jointUpdates[armJoints[1]] = pitch;
            this.recentJointAngles[hand].joint2 = pitch;
        }
        if (this.debug) {
            console.log(`${prefix}_joint2 (pitch after yaw adjustment): ${jointUpdates[armJoints[1]].toFixed(3)} rad`);
        }

        // Step 8: Compute elbow (joint3)
        const pitchAngle = jointUpdates[armJoints[1]];
        let wristTarget = target.clone().sub(new Vector3(linkLengths.wristToGripper, 0, 0));

        // Apply yaw and pitch rotations
        const cosPitch = Math.cos(pitchAngle);
        const sinPitch = Math.sin(pitchAngle);

        // Apply pitch rotation (around Y-axis)
        wristTarget = new Vector3(
            -wristTarget.x * -cosPitch - wristTarget.z * sinPitch,
            wristTarget.y,
            -wristTarget.x * sinPitch + wristTarget.z * cosPitch
        );

        if (this.debug) {
            console.log(`${prefix} wrist target in joint3 frame (yaw + pitch): x=${wristTarget.x.toFixed(3)}, y=${wristTarget.y.toFixed(3)}, z=${wristTarget.z.toFixed(3)}`);
        }

        // Clamp wrist target
        const wristDistXZ = Math.sqrt(wristTarget.x ** 2 + wristTarget.z ** 2 + 1e-10);
        const maxWristDist = linkLengths.upperArm + linkLengths.forearm + .01;
        if (wristDistXZ > maxWristDist) {
            const scaleXZ = maxWristDist / wristDistXZ;
            wristTarget.x *= scaleXZ;
            wristTarget.z *= scaleXZ;
            if (this.debug) {
                console.log(`${prefix} wrist target clamped: x=${wristTarget.x.toFixed(3)}, y=${wristTarget.y.toFixed(3)}, z=${wristTarget.z.toFixed(3)}`);
            }
        }

        // Compute elbow angle
        const joint3Control = jointControls.find(c => c.jointName === armJoints[2]);
        const L1 = linkLengths.upperArm;
        const L2 = linkLengths.forearm;
        const D = Math.sqrt(wristTarget.x ** 2 + wristTarget.z ** 2 + 1e-10);
        const cosElbow = Math.max(-0.999, Math.min(0.999,
            (L1 ** 2 + D ** 2 - L2 ** 2) / (2 * L1 * D)
        ));
        const elbowAngleCos = Math.acos(cosElbow);
        const wristAngle = Math.atan2(-wristTarget.z, D);
        let elbowAngle = wristAngle - elbowAngleCos;

        if (elbowAngle < joint3Control.lowerLimit || elbowAngle > joint3Control.upperLimit) {
            jointUpdates[armJoints[2]] = this.recentJointAngles[hand].joint3;
        } else {
            jointUpdates[armJoints[2]] = elbowAngle;
            this.recentJointAngles[hand].joint3 = elbowAngle;
        }
        if (this.debug) {
            console.log(`${prefix}_joint3 (elbow): ${jointUpdates[armJoints[2]].toFixed(3)} rad, wristDistXY=${wristDistXY.toFixed(3)}`);
        }

        // Step 9: Compute wrist joints (joint4, joint5, joint6) with yaw for orientation
        if (targetOrientation && targetOrientation instanceof Quaternion && isFinite(targetOrientation.x + targetOrientation.y + targetOrientation.z + targetOrientation.w)) {
            // Transform controller orientation to robot base frame
            const baseQuat = new Quaternion().setFromEuler(new Euler(-Math.PI / 2, 0, Math.PI / 2, 'XYZ'));
            const targetQuat = targetOrientation.clone().premultiply(baseQuat.clone().invert());

            // Compute the current arm orientation (after joint1, joint2, joint3)
            const yawQuat = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), jointUpdates[armJoints[0]]);
            const pitchQuat = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), jointUpdates[armJoints[1]] - pitchOffset);
            const elbowQuat = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), jointUpdates[armJoints[2]]);
            let armQuat = yawQuat.multiply(pitchQuat).multiply(elbowQuat);

            // Compute the relative orientation needed for the wrist
            const wristQuat = armQuat.clone().invert().multiply(targetQuat);

            // Convert wrist quaternion to Euler angles (XZY order for wrist joints)
            const wristEuler = new Euler().setFromQuaternion(wristQuat, 'XZY');
            const wristAngles = [
                wristEuler.y, // joint4
                wristEuler.z, // joint5
                wristEuler.x  // joint6
            ];

            wristAngles.forEach((angle, i) => {
                let adjustedAngle = angle;
                const jointName = armJoints[3 + i];
                jointUpdates[jointName] = adjustedAngle;

                if (this.debug) {
                    console.log(`${jointName} (wrist): ${jointUpdates[jointName].toFixed(3)} rad`);
                }
            });
        } else {
            // Fallback to zero angles if no valid orientation
            jointUpdates[armJoints[3]] = 0;
            jointUpdates[armJoints[4]] = 0;
            jointUpdates[armJoints[5]] = 0;
            if (this.debug) {
                console.log(`${prefix} wrist angles set to 0 (no valid orientation)`);
            }
        }

        // Step 10: Apply smoothing to joint angles
        const theta = armJoints.map((jointName, i) => {
            const angle = jointUpdates[jointName];
            const current = this.jointValues[jointName] || 0;
            if (!isFinite(angle)) {
                console.warn(`Invalid angle for ${jointName}: ${angle}`);
                return current;
            }
            const delta = angle - current;
            const maxDeltaForJoint = jointMaxDelta[jointName] || 1.0 * deltaTime;
            return Math.abs(delta) > maxDeltaForJoint ? current + Math.sign(delta) * maxDeltaForJoint : angle;
        });

        if (this.debug) {
            console.log(`${prefix} final joint angles: ${theta.map(a => a.toFixed(3)).join(', ')}`);
        }
        return theta;
    }

    // Controller input
    const controllers = {
                left: {
                    position: leftController.gripPosition ? new Vector3(
                        -leftController.gripPosition.x,
                        -leftController.gripPosition.y,
                        leftController.gripPosition.z
                    ) : null,
                    orientation: leftController.gripOrientation ? new Quaternion(
                        -leftController.gripOrientation.x,
                        leftController.gripOrientation.y,
                        leftController.gripOrientation.z,
                        leftController.gripOrientation.w
                    ) : null,
                    trigger: leftController.gamepad?.buttons?.[0]?.value || 0,
                    gripButton: leftController.gamepad?.buttons?.[1]?.pressed || false
                },
                right: {
                    position: rightController.gripPosition ? new Vector3(
                        rightController.gripPosition.x,
                        -rightController.gripPosition.y,
                        rightController.gripPosition.z
                    ) : null,
                    orientation: rightController.gripOrientation ? new Quaternion(
                        -rightController.gripOrientation.x,
                        rightController.gripOrientation.y,
                        rightController.gripOrientation.z,
                        rightController.gripOrientation.w
                    ) : null,
                    trigger: rightController.gamepad?.buttons?.[0]?.value || 0,
                    gripButton: rightController.gamepad?.buttons?.[1]?.pressed || false
                }
            };