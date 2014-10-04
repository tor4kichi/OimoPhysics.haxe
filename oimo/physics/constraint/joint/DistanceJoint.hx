
package oimo.physics.constraint.joint;

import oimo.physics.constraint.joint.Joint;
import oimo.physics.constraint.joint.JointConfig;

import oimo.math.Mat33;
import oimo.math.Vec3;
import oimo.physics.dynamics.RigidBody;
/**
	 * 剛体間の二点の距離を拘束するジョイントです。
	 * @author saharan
	 */
class DistanceJoint extends Joint
{
    /**
		 * 保とうとする剛体間の距離です。
		 */
    public var distance : Float;
    
    /**
		 * 拘束力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
    public var impulse : Float;
    
    private var posError : Float;
    
    private var lVel1 : Vec3;
    private var lVel2 : Vec3;
    
    private var aVel1 : Vec3;
    private var aVel2 : Vec3;
    
    private var norX : Float;
    private var norY : Float;
    private var norZ : Float;
    
    private var relPos1X : Float;
    private var relPos1Y : Float;
    private var relPos1Z : Float;
    private var relPos2X : Float;
    private var relPos2Y : Float;
    private var relPos2Z : Float;
    
    private var norTorque1X : Float;
    private var norTorque1Y : Float;
    private var norTorque1Z : Float;
    private var norTorque2X : Float;
    private var norTorque2Y : Float;
    private var norTorque2Z : Float;
    
    private var norTorqueUnit1X : Float;
    private var norTorqueUnit1Y : Float;
    private var norTorqueUnit1Z : Float;
    private var norTorqueUnit2X : Float;
    private var norTorqueUnit2Y : Float;
    private var norTorqueUnit2Z : Float;
    
    private var invM1 : Float;
    private var invM2 : Float;
    
    private var invI1e00 : Float;
    private var invI1e01 : Float;
    private var invI1e02 : Float;
    private var invI1e10 : Float;
    private var invI1e11 : Float;
    private var invI1e12 : Float;
    private var invI1e20 : Float;
    private var invI1e21 : Float;
    private var invI1e22 : Float;
    private var invI2e00 : Float;
    private var invI2e01 : Float;
    private var invI2e02 : Float;
    private var invI2e10 : Float;
    private var invI2e11 : Float;
    private var invI2e12 : Float;
    private var invI2e20 : Float;
    private var invI2e21 : Float;
    private var invI2e22 : Float;
    
    private var normalDenominator : Float;
    
    private var targetNormalVelocity : Float;
    
    /**
		 * 新しい DistanceJoint オブジェクトを作成します。
		 * @param	rigid1 剛体1
		 * @param	rigid2 剛体2
		 * @param	distance 保とうとする距離
		 * @param	config ジョイントの設定
		 */
    public function new(rigid1 : RigidBody, rigid2 : RigidBody, distance : Float, config : JointConfig)
    {
        super();
        this.body1 = rigid1;
        this.body2 = rigid2;
        connection1.connected = rigid2;
        connection2.connected = rigid1;
        this.distance = distance;
        allowCollision = config.allowCollision;
        localRelativeAnchorPosition1.copy(config.localRelativeAnchorPosition1);
        localRelativeAnchorPosition2.copy(config.localRelativeAnchorPosition2);
        type = JOINT_DISTANCE;
        
        lVel1 = this.body1.linearVelocity;
        lVel2 = this.body2.linearVelocity;
        aVel1 = this.body1.angularVelocity;
        aVel2 = this.body2.angularVelocity;
        
        invM1 = this.body1.invertMass;
        invM2 = this.body2.invertMass;
        
        impulse = 0;
    }
    
    /**
		 * @inheritDoc
		 */
    override public function preSolve(timeStep : Float, invTimeStep : Float) : Void{
        var tmpM : Mat33;
        var tmp1X : Float;
        var tmp1Y : Float;
        var tmp1Z : Float;
        var tmp2X : Float;
        var tmp2Y : Float;
        var tmp2Z : Float;
        
        // ----------------------------------------------
        //              calculate positions
        // ----------------------------------------------
        
        tmpM = body1.rotation;
        tmp1X = localRelativeAnchorPosition1.x;
        tmp1Y = localRelativeAnchorPosition1.y;
        tmp1Z = localRelativeAnchorPosition1.z;
        relPos1X = relativeAnchorPosition1.x = tmp1X * tmpM.e00 + tmp1Y * tmpM.e01 + tmp1Z * tmpM.e02;
        relPos1Y = relativeAnchorPosition1.y = tmp1X * tmpM.e10 + tmp1Y * tmpM.e11 + tmp1Z * tmpM.e12;
        relPos1Z = relativeAnchorPosition1.z = tmp1X * tmpM.e20 + tmp1Y * tmpM.e21 + tmp1Z * tmpM.e22;
        tmpM = body2.rotation;
        tmp1X = localRelativeAnchorPosition2.x;
        tmp1Y = localRelativeAnchorPosition2.y;
        tmp1Z = localRelativeAnchorPosition2.z;
        relPos2X = relativeAnchorPosition2.x = tmp1X * tmpM.e00 + tmp1Y * tmpM.e01 + tmp1Z * tmpM.e02;
        relPos2Y = relativeAnchorPosition2.y = tmp1X * tmpM.e10 + tmp1Y * tmpM.e11 + tmp1Z * tmpM.e12;
        relPos2Z = relativeAnchorPosition2.z = tmp1X * tmpM.e20 + tmp1Y * tmpM.e21 + tmp1Z * tmpM.e22;
        
        // ----------------------------------------------
        //               calculate normal
        // ----------------------------------------------
        
        norX = (anchorPosition2.x = relPos2X + body2.position.x) - (anchorPosition1.x = relPos1X + body1.position.x);
        norY = (anchorPosition2.y = relPos2Y + body2.position.y) - (anchorPosition1.y = relPos1Y + body1.position.y);
        norZ = (anchorPosition2.z = relPos2Z + body2.position.z) - (anchorPosition1.z = relPos1Z + body1.position.z);
        tmp1X = Math.sqrt(norX * norX + norY * norY + norZ * norZ);
        posError = distance - tmp1X;
        if (tmp1X > 0)             tmp1X = 1 / tmp1X;
        norX *= tmp1X;
        norY *= tmp1X;
        norZ *= tmp1X;
        
        // ----------------------------------------------
        // calculate torque axes and angular accelerations
        // ----------------------------------------------
        
        tmpM = body1.invertInertia;
        invI1e00 = tmpM.e00;
        invI1e01 = tmpM.e01;
        invI1e02 = tmpM.e02;
        invI1e10 = tmpM.e10;
        invI1e11 = tmpM.e11;
        invI1e12 = tmpM.e12;
        invI1e20 = tmpM.e20;
        invI1e21 = tmpM.e21;
        invI1e22 = tmpM.e22;
        tmpM = body2.invertInertia;
        invI2e00 = tmpM.e00;
        invI2e01 = tmpM.e01;
        invI2e02 = tmpM.e02;
        invI2e10 = tmpM.e10;
        invI2e11 = tmpM.e11;
        invI2e12 = tmpM.e12;
        invI2e20 = tmpM.e20;
        invI2e21 = tmpM.e21;
        invI2e22 = tmpM.e22;
        
        norTorque1X = relPos1Y * norZ - relPos1Z * norY;
        norTorque1Y = relPos1Z * norX - relPos1X * norZ;
        norTorque1Z = relPos1X * norY - relPos1Y * norX;
        norTorque2X = relPos2Y * norZ - relPos2Z * norY;
        norTorque2Y = relPos2Z * norX - relPos2X * norZ;
        norTorque2Z = relPos2X * norY - relPos2Y * norX;
        
        norTorqueUnit1X = norTorque1X * invI1e00 + norTorque1Y * invI1e01 + norTorque1Z * invI1e02;
        norTorqueUnit1Y = norTorque1X * invI1e10 + norTorque1Y * invI1e11 + norTorque1Z * invI1e12;
        norTorqueUnit1Z = norTorque1X * invI1e20 + norTorque1Y * invI1e21 + norTorque1Z * invI1e22;
        norTorqueUnit2X = norTorque2X * invI2e00 + norTorque2Y * invI2e01 + norTorque2Z * invI2e02;
        norTorqueUnit2Y = norTorque2X * invI2e10 + norTorque2Y * invI2e11 + norTorque2Z * invI2e12;
        norTorqueUnit2Z = norTorque2X * invI2e20 + norTorque2Y * invI2e21 + norTorque2Z * invI2e22;
        
        // ----------------------------------------------
        //         calculate impulse denominator
        // ----------------------------------------------
        
        tmp1X = norTorque1X * invI1e00 + norTorque1Y * invI1e01 + norTorque1Z * invI1e02;
        tmp1Y = norTorque1X * invI1e10 + norTorque1Y * invI1e11 + norTorque1Z * invI1e12;
        tmp1Z = norTorque1X * invI1e20 + norTorque1Y * invI1e21 + norTorque1Z * invI1e22;
        tmp2X = tmp1Y * relPos1Z - tmp1Z * relPos1Y;
        tmp2Y = tmp1Z * relPos1X - tmp1X * relPos1Z;
        tmp2Z = tmp1X * relPos1Y - tmp1Y * relPos1X;
        tmp1X = norTorque2X * invI2e00 + norTorque2Y * invI2e01 + norTorque2Z * invI2e02;
        tmp1Y = norTorque2X * invI2e10 + norTorque2Y * invI2e11 + norTorque2Z * invI2e12;
        tmp1Z = norTorque2X * invI2e20 + norTorque2Y * invI2e21 + norTorque2Z * invI2e22;
        tmp2X += tmp1Y * relPos2Z - tmp1Z * relPos2Y;
        tmp2Y += tmp1Z * relPos2X - tmp1X * relPos2Z;
        tmp2Z += tmp1X * relPos2Y - tmp1Y * relPos2X;
        normalDenominator = 1 / (invM1 + invM2 + norX * tmp2X + norY * tmp2Y + norZ * tmp2Z);
        
        // ----------------------------------------------
        //           calculate initial forces
        // ----------------------------------------------
        
        tmp1X = norX * impulse;
        tmp1Y = norY * impulse;
        tmp1Z = norZ * impulse;
        lVel1.x += tmp1X * invM1;
        lVel1.y += tmp1Y * invM1;
        lVel1.z += tmp1Z * invM1;
        aVel1.x += norTorqueUnit1X * impulse;
        aVel1.y += norTorqueUnit1Y * impulse;
        aVel1.z += norTorqueUnit1Z * impulse;
        lVel2.x -= tmp1X * invM2;
        lVel2.y -= tmp1Y * invM2;
        lVel2.z -= tmp1Z * invM2;
        aVel2.x -= norTorqueUnit2X * impulse;
        aVel2.y -= norTorqueUnit2Y * impulse;
        aVel2.z -= norTorqueUnit2Z * impulse;
        
        // ----------------------------------------------
        //           calculate target velocity
        // ----------------------------------------------
        
        if (posError < -0.005)             posError += 0.005
        // allow 0.5cm error
        else if (posError > 0.005)             posError -= 0.005
        else posError = 0;
        targetNormalVelocity = posError * invTimeStep * 0.05;
    }
    
    /**
		 * @inheritDoc
		 */
    override public function solve() : Void{
        var newImpulse : Float;
        var rvn : Float;
        var forceX : Float;
        var forceY : Float;
        var forceZ : Float;
        var tmpX : Float;
        var tmpY : Float;
        var tmpZ : Float;
        rvn =
                (lVel2.x - lVel1.x) * norX + (lVel2.y - lVel1.y) * norY + (lVel2.z - lVel1.z) * norZ +
                aVel2.x * norTorque2X + aVel2.y * norTorque2Y + aVel2.z * norTorque2Z -
                aVel1.x * norTorque1X - aVel1.y * norTorque1Y - aVel1.z * norTorque1Z;
        newImpulse = (rvn - targetNormalVelocity) * normalDenominator;
        impulse += newImpulse;
        forceX = norX * newImpulse;
        forceY = norY * newImpulse;
        forceZ = norZ * newImpulse;
        lVel1.x += forceX * invM1;
        lVel1.y += forceY * invM1;
        lVel1.z += forceZ * invM1;
        aVel1.x += norTorqueUnit1X * newImpulse;
        aVel1.y += norTorqueUnit1Y * newImpulse;
        aVel1.z += norTorqueUnit1Z * newImpulse;
        lVel2.x -= forceX * invM2;
        lVel2.y -= forceY * invM2;
        lVel2.z -= forceZ * invM2;
        aVel2.x -= norTorqueUnit2X * newImpulse;
        aVel2.y -= norTorqueUnit2Y * newImpulse;
        aVel2.z -= norTorqueUnit2Z * newImpulse;
    }
    
    /**
		 * @inheritDoc
		 */
    override public function postSolve() : Void{
        
    }
}

