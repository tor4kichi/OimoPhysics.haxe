/* Copyright (c) 2012 EL-EMENT saharan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation  * files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy,  * modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package oimo.physics.constraint.joint;

import oimo.physics.constraint.joint.Joint;
import oimo.physics.constraint.joint.JointConfig;

import oimo.math.Mat33;
import oimo.math.Vec3;
import oimo.physics.dynamics.RigidBody;
/**
	 * 剛体間の回転を軸方向のみに拘束するジョイントです。
	 * 蝶番のような動きをします。
	 * @author saharan
	 */
class HingeJoint extends Joint
{
    /**
		 * 並進速度の拘束力のベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
    public var impulse : Vec3;
    
    /**
		 * 可動範囲の制限に使用した拘束力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
    public var limitTorque : Float;
    
    /**
		 * モーターに使用した拘束力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
    public var motorTorque : Float;
    
    /**
		 * 角速度の拘束力のベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
    public var torque : Vec3;
    
    /**
		 * 剛体1の回転の軸を表すベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var localAxis1 : Vec3;
    
    /**
		 * 剛体2の回転の軸を表すベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var localAxis2 : Vec3;
    
    /**
		 * 可動範囲の制限が有効であるかどうかを表します。
		 */
    public var enableLimits : Bool;
    
    /**
		 * 可動範囲の下限です。
		 * enableLimits が true の時にのみ有効になります。
		 */
    public var lowerLimit : Float;
    
    /**
		 * 可動範囲の上限です。
		 * enableLimits が true の時にのみ有効になります。
		 */
    public var upperLimit : Float;
    
    private var limitSign : Int;
    
    /**
		 * モーターが有効であるかどうかを表します。
		 */
    public var enableMotor : Bool;
    
    /**
		 * モーターの速度です。
		 * enableMotor が true の時にのみ有効になります。
		 */
    public var motorSpeed : Float;
    
    /**
		 * モーターの最大のトルクです。
		 * enableMotor が true の時にのみ有効になります。
		 */
    public var maxMotorTorque : Float;
    
    private var stepMotorTorque : Float;
    
    private var impulseX : Float;
    private var impulseY : Float;
    private var impulseZ : Float;
    
    private var torqueX : Float;
    private var torqueY : Float;
    private var torqueZ : Float;
    
    private var torqueTan : Float;
    private var torqueBin : Float;
    
    private var localAxis1X : Float;
    private var localAxis1Y : Float;
    private var localAxis1Z : Float;
    
    private var localAxis2X : Float;
    private var localAxis2Y : Float;
    private var localAxis2Z : Float;
    
    private var localAngAxis1X : Float;
    private var localAngAxis1Y : Float;
    private var localAngAxis1Z : Float;
    
    private var localAngAxis2X : Float;
    private var localAngAxis2Y : Float;
    private var localAngAxis2Z : Float;
    
    private var axis1X : Float;
    private var axis1Y : Float;
    private var axis1Z : Float;
    
    private var axis2X : Float;
    private var axis2Y : Float;
    private var axis2Z : Float;
    
    private var angAxis1X : Float;
    private var angAxis1Y : Float;
    private var angAxis1Z : Float;
    
    private var angAxis2X : Float;
    private var angAxis2Y : Float;
    private var angAxis2Z : Float;
    
    private var norX : Float;
    private var norY : Float;
    private var norZ : Float;
    
    private var tanX : Float;
    private var tanY : Float;
    private var tanZ : Float;
    
    private var binX : Float;
    private var binY : Float;
    private var binZ : Float;
    
    private var hingeAngle : Float;
    
    private var lVel1 : Vec3;
    private var lVel2 : Vec3;
    
    private var aVel1 : Vec3;
    private var aVel2 : Vec3;
    
    private var relPos1X : Float;
    private var relPos1Y : Float;
    private var relPos1Z : Float;
    private var relPos2X : Float;
    private var relPos2Y : Float;
    private var relPos2Z : Float;
    
    private var xTorqueUnit1X : Float;
    private var xTorqueUnit1Y : Float;
    private var xTorqueUnit1Z : Float;
    private var xTorqueUnit2X : Float;
    private var xTorqueUnit2Y : Float;
    private var xTorqueUnit2Z : Float;
    
    private var yTorqueUnit1X : Float;
    private var yTorqueUnit1Y : Float;
    private var yTorqueUnit1Z : Float;
    private var yTorqueUnit2X : Float;
    private var yTorqueUnit2Y : Float;
    private var yTorqueUnit2Z : Float;
    
    private var zTorqueUnit1X : Float;
    private var zTorqueUnit1Y : Float;
    private var zTorqueUnit1Z : Float;
    private var zTorqueUnit2X : Float;
    private var zTorqueUnit2Y : Float;
    private var zTorqueUnit2Z : Float;
    
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
    
    // denominator matrix (Point-To-Point mass matrix)
    private var d00 : Float;
    private var d01 : Float;
    private var d02 : Float;
    private var d10 : Float;
    private var d11 : Float;
    private var d12 : Float;
    private var d20 : Float;
    private var d21 : Float;
    private var d22 : Float;
    
    private var norDenominator : Float;
    private var tanDenominator : Float;
    private var binDenominator : Float;
    
    private var invNorDenominator : Float;
    
    private var targetVelX : Float;
    private var targetVelY : Float;
    private var targetVelZ : Float;
    
    private var targetAngVelNor : Float;
    private var targetAngVelTan : Float;
    private var targetAngVelBin : Float;
    
    /**
		 * 新しい HingeJoint オブジェクトを作成します。
		 * @param	rigid1 剛体1
		 * @param	rigid2 剛体2
		 * @param	config ジョイントの設定
		 */
    public function new(rigid1 : RigidBody, rigid2 : RigidBody, config : JointConfig)
    {
        super();
        this.body1 = rigid1;
        this.body2 = rigid2;
        connection1.connected = rigid2;
        connection2.connected = rigid1;
        localAxis1 = new Vec3();
        localAxis2 = new Vec3();
        localAxis1.normalize(config.localAxis1);
        localAxis2.normalize(config.localAxis2);
        
        var len : Float;
        localAxis1X = localAxis1.x;
        localAxis1Y = localAxis1.y;
        localAxis1Z = localAxis1.z;
        localAngAxis1X = localAxis1Y * localAxis1X - localAxis1Z * localAxis1Z;
        localAngAxis1Y = -localAxis1Z * localAxis1Y - localAxis1X * localAxis1X;
        localAngAxis1Z = localAxis1X * localAxis1Z + localAxis1Y * localAxis1Y;
        len = 1 / Math.sqrt(localAngAxis1X * localAngAxis1X + localAngAxis1Y * localAngAxis1Y + localAngAxis1Z * localAngAxis1Z);
        localAngAxis1X *= len;
        localAngAxis1Y *= len;
        localAngAxis1Z *= len;
        localAxis2X = localAxis2.x;
        localAxis2Y = localAxis2.y;
        localAxis2Z = localAxis2.z;
        localAngAxis2X = localAxis2Y * localAxis2X - localAxis2Z * localAxis2Z;
        localAngAxis2Y = -localAxis2Z * localAxis2Y - localAxis2X * localAxis2X;
        localAngAxis2Z = localAxis2X * localAxis2Z + localAxis2Y * localAxis2Y;
        len = 1 / Math.sqrt(localAngAxis2X * localAngAxis2X + localAngAxis2Y * localAngAxis2Y + localAngAxis2Z * localAngAxis2Z);
        localAngAxis2X *= len;
        localAngAxis2Y *= len;
        localAngAxis2Z *= len;
        
        allowCollision = config.allowCollision;
        localRelativeAnchorPosition1.copy(config.localRelativeAnchorPosition1);
        localRelativeAnchorPosition2.copy(config.localRelativeAnchorPosition2);
        type = JOINT_HINGE;
        
        lVel1 = this.body1.linearVelocity;
        lVel2 = this.body2.linearVelocity;
        aVel1 = this.body1.angularVelocity;
        aVel2 = this.body2.angularVelocity;
        
        invM1 = this.body1.invertMass;
        invM2 = this.body2.invertMass;
        
        impulse = new Vec3();
        torque = new Vec3();
        impulseX = 0;
        impulseY = 0;
        impulseZ = 0;
        limitTorque = 0;
        motorTorque = 0;
        torqueX = 0;
        torqueY = 0;
        torqueZ = 0;
        torqueTan = 0;
        torqueBin = 0;
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
        var t00 : Float;
        var t01 : Float;
        var t02 : Float;
        var t10 : Float;
        var t11 : Float;
        var t12 : Float;
        var t20 : Float;
        var t21 : Float;
        var t22 : Float;
        var u00 : Float;
        var u01 : Float;
        var u02 : Float;
        var u10 : Float;
        var u11 : Float;
        var u12 : Float;
        var u20 : Float;
        var u21 : Float;
        var u22 : Float;
        
        // ----------------------------------------------
        //              calculate positions
        // ----------------------------------------------
        
        tmpM = body1.rotation;
        axis1X = localAxis1X * tmpM.e00 + localAxis1Y * tmpM.e01 + localAxis1Z * tmpM.e02;
        axis1Y = localAxis1X * tmpM.e10 + localAxis1Y * tmpM.e11 + localAxis1Z * tmpM.e12;
        axis1Z = localAxis1X * tmpM.e20 + localAxis1Y * tmpM.e21 + localAxis1Z * tmpM.e22;
        angAxis1X = localAngAxis1X * tmpM.e00 + localAngAxis1Y * tmpM.e01 + localAngAxis1Z * tmpM.e02;
        angAxis1Y = localAngAxis1X * tmpM.e10 + localAngAxis1Y * tmpM.e11 + localAngAxis1Z * tmpM.e12;
        angAxis1Z = localAngAxis1X * tmpM.e20 + localAngAxis1Y * tmpM.e21 + localAngAxis1Z * tmpM.e22;
        tmp1X = localRelativeAnchorPosition1.x;
        tmp1Y = localRelativeAnchorPosition1.y;
        tmp1Z = localRelativeAnchorPosition1.z;
        relPos1X = relativeAnchorPosition1.x = tmp1X * tmpM.e00 + tmp1Y * tmpM.e01 + tmp1Z * tmpM.e02;
        relPos1Y = relativeAnchorPosition1.y = tmp1X * tmpM.e10 + tmp1Y * tmpM.e11 + tmp1Z * tmpM.e12;
        relPos1Z = relativeAnchorPosition1.z = tmp1X * tmpM.e20 + tmp1Y * tmpM.e21 + tmp1Z * tmpM.e22;
        tmpM = body2.rotation;
        axis2X = localAxis2X * tmpM.e00 + localAxis2Y * tmpM.e01 + localAxis2Z * tmpM.e02;
        axis2Y = localAxis2X * tmpM.e10 + localAxis2Y * tmpM.e11 + localAxis2Z * tmpM.e12;
        axis2Z = localAxis2X * tmpM.e20 + localAxis2Y * tmpM.e21 + localAxis2Z * tmpM.e22;
        angAxis2X = localAngAxis2X * tmpM.e00 + localAngAxis2Y * tmpM.e01 + localAngAxis2Z * tmpM.e02;
        angAxis2Y = localAngAxis2X * tmpM.e10 + localAngAxis2Y * tmpM.e11 + localAngAxis2Z * tmpM.e12;
        angAxis2Z = localAngAxis2X * tmpM.e20 + localAngAxis2Y * tmpM.e21 + localAngAxis2Z * tmpM.e22;
        tmp1X = localRelativeAnchorPosition2.x;
        tmp1Y = localRelativeAnchorPosition2.y;
        tmp1Z = localRelativeAnchorPosition2.z;
        relPos2X = relativeAnchorPosition2.x = tmp1X * tmpM.e00 + tmp1Y * tmpM.e01 + tmp1Z * tmpM.e02;
        relPos2Y = relativeAnchorPosition2.y = tmp1X * tmpM.e10 + tmp1Y * tmpM.e11 + tmp1Z * tmpM.e12;
        relPos2Z = relativeAnchorPosition2.z = tmp1X * tmpM.e20 + tmp1Y * tmpM.e21 + tmp1Z * tmpM.e22;
        anchorPosition1.x = relPos1X + body1.position.x;
        anchorPosition1.y = relPos1Y + body1.position.y;
        anchorPosition1.z = relPos1Z + body1.position.z;
        anchorPosition2.x = relPos2X + body2.position.x;
        anchorPosition2.y = relPos2Y + body2.position.y;
        anchorPosition2.z = relPos2Z + body2.position.z;
        tmp1X = 1 / (invM1 + invM2);
        norX = (axis1X * invM1 + axis2X * invM2) * tmp1X;
        norY = (axis1Y * invM1 + axis2Y * invM2) * tmp1X;
        norZ = (axis1Z * invM1 + axis2Z * invM2) * tmp1X;
        tmp1X = Math.sqrt(norX * norX + norY * norY + norZ * norZ);
        if (tmp1X > 0)             tmp1X = 1 / tmp1X;
        norX *= tmp1X;
        norY *= tmp1X;
        norZ *= tmp1X;
        tanX = norY * norX - norZ * norZ;
        tanY = -norZ * norY - norX * norX;
        tanZ = norX * norZ + norY * norY;
        tmp1X = 1 / Math.sqrt(tanX * tanX + tanY * tanY + tanZ * tanZ);
        tanX *= tmp1X;
        tanY *= tmp1X;
        tanZ *= tmp1X;
        binX = norY * tanZ - norZ * tanY;
        binY = norZ * tanX - norX * tanZ;
        binZ = norX * tanY - norY * tanX;
        
        // ----------------------------------------------
        //            calculate hinge angle
        // ----------------------------------------------
        
        if (
            norX != 0 * (angAxis1Y != 0 * angAxis2Z != 0 - angAxis1Z != 0 * angAxis2Y != 0) +
            norY != 0 * (angAxis1Z != 0 * angAxis2X != 0 - angAxis1X != 0 * angAxis2Z != 0) +
            norZ * (angAxis1X * angAxis2Y - angAxis1Y * angAxis2X) <0  // cross product  ) {
            hingeAngle = -Math.acos(angAxis1X * angAxis2X + angAxis1Y * angAxis2Y + angAxis1Z * angAxis2Z);
        }
        else {
            hingeAngle = Math.acos(angAxis1X * angAxis2X + angAxis1Y * angAxis2Y + angAxis1Z * angAxis2Z);
        }  // ----------------------------------------------    //        calculate angular accelerations    // ----------------------------------------------  
        
        
        
        
        
        
        
        
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
        
        xTorqueUnit1X = relPos1Z * invI1e01 - relPos1Y * invI1e02;
        xTorqueUnit1Y = relPos1Z * invI1e11 - relPos1Y * invI1e12;
        xTorqueUnit1Z = relPos1Z * invI1e21 - relPos1Y * invI1e22;
        xTorqueUnit2X = relPos2Z * invI2e01 - relPos2Y * invI2e02;
        xTorqueUnit2Y = relPos2Z * invI2e11 - relPos2Y * invI2e12;
        xTorqueUnit2Z = relPos2Z * invI2e21 - relPos2Y * invI2e22;
        
        yTorqueUnit1X = -relPos1Z * invI1e00 + relPos1X * invI1e02;
        yTorqueUnit1Y = -relPos1Z * invI1e10 + relPos1X * invI1e12;
        yTorqueUnit1Z = -relPos1Z * invI1e20 + relPos1X * invI1e22;
        yTorqueUnit2X = -relPos2Z * invI2e00 + relPos2X * invI2e02;
        yTorqueUnit2Y = -relPos2Z * invI2e10 + relPos2X * invI2e12;
        yTorqueUnit2Z = -relPos2Z * invI2e20 + relPos2X * invI2e22;
        
        zTorqueUnit1X = relPos1Y * invI1e00 - relPos1X * invI1e01;
        zTorqueUnit1Y = relPos1Y * invI1e10 - relPos1X * invI1e11;
        zTorqueUnit1Z = relPos1Y * invI1e20 - relPos1X * invI1e21;
        zTorqueUnit2X = relPos2Y * invI2e00 - relPos2X * invI2e01;
        zTorqueUnit2Y = relPos2Y * invI2e10 - relPos2X * invI2e11;
        zTorqueUnit2Z = relPos2Y * invI2e20 - relPos2X * invI2e21;
        
        // ----------------------------------------------
        //         calculate impulse denominator
        // ----------------------------------------------
        
        // calculate Point-To-Point mass matrix
        // from impulse equation
        //
        // M = ([/m] - [r^][/I][r^]) ^ -1
        //
        // where:
        //
        // [/m] = |1/m, 0, 0|
        //        |0, 1/m, 0|
        //        |0, 0, 1/m|
        //
        // [r^] = |0, -rz, ry|
        //        |rz, 0, -rx|
        //        |-ry, rx, 0|
        //
        // [/I] = Inverted moment inertia
        
        d00 = invM1 + invM2;
        d01 = 0;
        d02 = 0;
        d10 = 0;
        d11 = d00;
        d12 = 0;
        d20 = 0;
        d21 = 0;
        d22 = d00;
        t01 = -relPos1Z;
        t02 = relPos1Y;
        t10 = relPos1Z;
        t12 = -relPos1X;
        t20 = -relPos1Y;
        t21 = relPos1X;
        u00 = invI1e01 * t10 + invI1e02 * t20;
        u01 = invI1e00 * t01 + invI1e02 * t21;
        u02 = invI1e00 * t02 + invI1e01 * t12;
        u10 = invI1e11 * t10 + invI1e12 * t20;
        u11 = invI1e10 * t01 + invI1e12 * t21;
        u12 = invI1e10 * t02 + invI1e11 * t12;
        u20 = invI1e21 * t10 + invI1e22 * t20;
        u21 = invI1e20 * t01 + invI1e22 * t21;
        u22 = invI1e20 * t02 + invI1e21 * t12;
        d00 -= t01 * u10 + t02 * u20;
        d01 -= t01 * u11 + t02 * u21;
        d02 -= t01 * u12 + t02 * u22;
        d10 -= t10 * u00 + t12 * u20;
        d11 -= t10 * u01 + t12 * u21;
        d12 -= t10 * u02 + t12 * u22;
        d20 -= t20 * u00 + t21 * u10;
        d21 -= t20 * u01 + t21 * u11;
        d22 -= t20 * u02 + t21 * u12;
        t01 = -relPos2Z;
        t02 = relPos2Y;
        t10 = relPos2Z;
        t12 = -relPos2X;
        t20 = -relPos2Y;
        t21 = relPos2X;
        u00 = invI2e01 * t10 + invI2e02 * t20;
        u01 = invI2e00 * t01 + invI2e02 * t21;
        u02 = invI2e00 * t02 + invI2e01 * t12;
        u10 = invI2e11 * t10 + invI2e12 * t20;
        u11 = invI2e10 * t01 + invI2e12 * t21;
        u12 = invI2e10 * t02 + invI2e11 * t12;
        u20 = invI2e21 * t10 + invI2e22 * t20;
        u21 = invI2e20 * t01 + invI2e22 * t21;
        u22 = invI2e20 * t02 + invI2e21 * t12;
        d00 -= t01 * u10 + t02 * u20;
        d01 -= t01 * u11 + t02 * u21;
        d02 -= t01 * u12 + t02 * u22;
        d10 -= t10 * u00 + t12 * u20;
        d11 -= t10 * u01 + t12 * u21;
        d12 -= t10 * u02 + t12 * u22;
        d20 -= t20 * u00 + t21 * u10;
        d21 -= t20 * u01 + t21 * u11;
        d22 -= t20 * u02 + t21 * u12;
        tmp1X = 1 / (d00 * (d11 * d22 - d21 * d12) + d10 * (d21 * d02 - d01 * d22) + d20 * (d01 * d12 - d11 * d02));
        t00 = (d11 * d22 - d12 * d21) * tmp1X;
        t01 = (d02 * d21 - d01 * d22) * tmp1X;
        t02 = (d01 * d12 - d02 * d11) * tmp1X;
        t10 = (d12 * d20 - d10 * d22) * tmp1X;
        t11 = (d00 * d22 - d02 * d20) * tmp1X;
        t12 = (d02 * d10 - d00 * d12) * tmp1X;
        t20 = (d10 * d21 - d11 * d20) * tmp1X;
        t21 = (d01 * d20 - d00 * d21) * tmp1X;
        t22 = (d00 * d11 - d01 * d10) * tmp1X;
        d00 = t00;
        d01 = t01;
        d02 = t02;
        d10 = t10;
        d11 = t11;
        d12 = t12;
        d20 = t20;
        d21 = t21;
        d22 = t22;
        
        t00 = invI1e00 + invI2e00;
        t01 = invI1e01 + invI2e01;
        t02 = invI1e02 + invI2e02;
        t10 = invI1e10 + invI2e10;
        t11 = invI1e11 + invI2e11;
        t12 = invI1e12 + invI2e12;
        t20 = invI1e20 + invI2e20;
        t21 = invI1e21 + invI2e21;
        t22 = invI1e22 + invI2e22;
        
        invNorDenominator =
                norX * (norX * t00 + norY * t01 + norZ * t02) +
                norY * (norX * t10 + norY * t11 + norZ * t12) +
                norZ * (norX * t20 + norY * t21 + norZ * t22);
        norDenominator = 1 / invNorDenominator;
        
        tanDenominator =
                1 / (
                tanX * (tanX * t00 + tanY * t01 + tanZ * t02) +
                tanY * (tanX * t10 + tanY * t11 + tanZ * t12) +
                tanZ * (tanX * t20 + tanY * t21 + tanZ * t22));
        
        binDenominator =
                1 / (
                binX * (binX * t00 + binY * t01 + binZ * t02) +
                binY * (binX * t10 + binY * t11 + binZ * t12) +
                binZ * (binX * t20 + binY * t21 + binZ * t22));
        
        // ----------------------------------------------
        //           calculate limits and motor
        // ----------------------------------------------
        
        // limit error
        if (enableLimits) {
            if (hingeAngle < lowerLimit) {
                if (limitSign != -1)                     limitTorque = 0;
                limitSign = -1;
            }
            else if (hingeAngle > upperLimit) {
                if (limitSign != 1)                     limitTorque = 0;
                limitSign = 1;
            }
            else {
                limitSign = 0;
                limitTorque = 0;
            }
        }
        else {
            limitSign = 0;
            limitTorque = 0;
        }
        
        if (enableMotor) {
            stepMotorTorque = timeStep * maxMotorTorque;
        }  // ----------------------------------------------    //           calculate initial forces    // ----------------------------------------------  
        
        
        
        
        
        
        
        
        torqueTan *= 0.95;
        torqueBin *= 0.95;
        motorTorque *= 0.95;
        limitTorque *= 0.95;
        
        tmp1X = limitTorque + motorTorque;
        torqueX = tmp1X * norX + torqueTan * tanX + torqueBin * binX;
        torqueY = tmp1X * norY + torqueTan * tanY + torqueBin * binY;
        torqueZ = tmp1X * norZ + torqueTan * tanZ + torqueBin * binZ;
        
        lVel1.x += impulseX * invM1;
        lVel1.y += impulseY * invM1;
        lVel1.z += impulseZ * invM1;
        aVel1.x += xTorqueUnit1X * impulseX + yTorqueUnit1X * impulseY + zTorqueUnit1X * impulseZ + torqueX * invI1e00 + torqueY * invI1e01 + torqueZ * invI1e02;
        aVel1.y += xTorqueUnit1Y * impulseX + yTorqueUnit1Y * impulseY + zTorqueUnit1Y * impulseZ + torqueX * invI1e10 + torqueY * invI1e11 + torqueZ * invI1e12;
        aVel1.z += xTorqueUnit1Z * impulseX + yTorqueUnit1Z * impulseY + zTorqueUnit1Z * impulseZ + torqueX * invI1e20 + torqueY * invI1e21 + torqueZ * invI1e22;
        lVel2.x -= impulseX * invM2;
        lVel2.y -= impulseY * invM2;
        lVel2.z -= impulseZ * invM2;
        aVel2.x -= xTorqueUnit2X * impulseX + yTorqueUnit2X * impulseY + zTorqueUnit2X * impulseZ + torqueX * invI2e00 + torqueY * invI2e01 + torqueZ * invI2e02;
        aVel2.y -= xTorqueUnit2Y * impulseX + yTorqueUnit2Y * impulseY + zTorqueUnit2Y * impulseZ + torqueX * invI2e10 + torqueY * invI2e11 + torqueZ * invI2e12;
        aVel2.z -= xTorqueUnit2Z * impulseX + yTorqueUnit2Z * impulseY + zTorqueUnit2Z * impulseZ + torqueX * invI2e20 + torqueY * invI2e21 + torqueZ * invI2e22;
        
        // ----------------------------------------------
        //           calculate target velocity
        // ----------------------------------------------
        
        // linear error
        targetVelX = anchorPosition2.x - anchorPosition1.x;
        targetVelY = anchorPosition2.y - anchorPosition1.y;
        targetVelZ = anchorPosition2.z - anchorPosition1.z;
        tmp1X = Math.sqrt(targetVelX * targetVelX + targetVelY * targetVelY + targetVelZ * targetVelZ);
        if (tmp1X < 0.005) {
            targetVelX = 0;
            targetVelY = 0;
            targetVelZ = 0;
        }
        else {
            tmp1X = (0.005 - tmp1X) / tmp1X * invTimeStep * 0.05;
            targetVelX *= tmp1X;
            targetVelY *= tmp1X;
            targetVelZ *= tmp1X;
        }  // angular error  
        
        
        
        tmp1X = axis2Y * axis1Z - axis2Z * axis1Y;
        tmp1Y = axis2Z * axis1X - axis2X * axis1Z;
        tmp1Z = axis2X * axis1Y - axis2Y * axis1X;
        
        targetAngVelTan = tanX * tmp1X + tanY * tmp1Y + tanZ * tmp1Z;
        if (targetAngVelTan > 0.02)             targetAngVelTan = (targetAngVelTan - 0.02) * invTimeStep * 0.05
        else if (targetAngVelTan < -0.02)             targetAngVelTan = (targetAngVelTan + 0.02) * invTimeStep * 0.05
        else targetAngVelTan = 0;
        
        targetAngVelBin = binX * tmp1X + binY * tmp1Y + binZ * tmp1Z;
        if (targetAngVelBin > 0.02)             targetAngVelBin = (targetAngVelBin - 0.02) * invTimeStep * 0.05
        else if (targetAngVelBin < -0.02)             targetAngVelBin = (targetAngVelBin + 0.02) * invTimeStep * 0.05
        else targetAngVelBin = 0;
        
        // limit error
        if (limitSign == -1) {
            targetAngVelNor = lowerLimit - hingeAngle;
            if (targetAngVelNor < 0.02)                 targetAngVelNor = 0
            else targetAngVelNor = (targetAngVelNor - 0.02) * invTimeStep * 0.05;
        }
        else if (limitSign == 1) {
            targetAngVelNor = upperLimit - hingeAngle;
            if (targetAngVelNor > -0.02)                 targetAngVelNor = 0
            else targetAngVelNor = (targetAngVelNor + 0.02) * invTimeStep * 0.05;
        }
        else {
            targetAngVelNor = 0;
        }
    }
    
    /**
		 * @inheritDoc
		 */
    override public function solve() : Void{
        var relVelX : Float;
        var relVelY : Float;
        var relVelZ : Float;
        var tmp : Float;
        var newImpulseX : Float;
        var newImpulseY : Float;
        var newImpulseZ : Float;
        var newTorqueTan : Float;
        var newTorqueBin : Float;
        var oldMotorTorque : Float;
        var newMotorTorque : Float;
        var oldLimitTorque : Float;
        var newLimitTorque : Float;
        
        relVelX = lVel2.x - lVel1.x + aVel2.y * relPos2Z - aVel2.z * relPos2Y - aVel1.y * relPos1Z + aVel1.z * relPos1Y - targetVelX;
        relVelY = lVel2.y - lVel1.y + aVel2.z * relPos2X - aVel2.x * relPos2Z - aVel1.z * relPos1X + aVel1.x * relPos1Z - targetVelY;
        relVelZ = lVel2.z - lVel1.z + aVel2.x * relPos2Y - aVel2.y * relPos2X - aVel1.x * relPos1Y + aVel1.y * relPos1X - targetVelZ;
        newImpulseX = relVelX * d00 + relVelY * d01 + relVelZ * d02;
        newImpulseY = relVelX * d10 + relVelY * d11 + relVelZ * d12;
        newImpulseZ = relVelX * d20 + relVelY * d21 + relVelZ * d22;
        impulseX += newImpulseX;
        impulseY += newImpulseY;
        impulseZ += newImpulseZ;
        lVel1.x += newImpulseX * invM1;
        lVel1.y += newImpulseY * invM1;
        lVel1.z += newImpulseZ * invM1;
        aVel1.x += xTorqueUnit1X * newImpulseX + yTorqueUnit1X * newImpulseY + zTorqueUnit1X * newImpulseZ;
        aVel1.y += xTorqueUnit1Y * newImpulseX + yTorqueUnit1Y * newImpulseY + zTorqueUnit1Y * newImpulseZ;
        aVel1.z += xTorqueUnit1Z * newImpulseX + yTorqueUnit1Z * newImpulseY + zTorqueUnit1Z * newImpulseZ;
        lVel2.x -= newImpulseX * invM2;
        lVel2.y -= newImpulseY * invM2;
        lVel2.z -= newImpulseZ * invM2;
        aVel2.x -= xTorqueUnit2X * newImpulseX + yTorqueUnit2X * newImpulseY + zTorqueUnit2X * newImpulseZ;
        aVel2.y -= xTorqueUnit2Y * newImpulseX + yTorqueUnit2Y * newImpulseY + zTorqueUnit2Y * newImpulseZ;
        aVel2.z -= xTorqueUnit2Z * newImpulseX + yTorqueUnit2Z * newImpulseY + zTorqueUnit2Z * newImpulseZ;
        
        relVelX = aVel2.x - aVel1.x;
        relVelY = aVel2.y - aVel1.y;
        relVelZ = aVel2.z - aVel1.z;
        tmp = relVelX * norX + relVelY * norY + relVelZ * norZ;
        
        if (enableMotor) {
            newMotorTorque = (tmp - motorSpeed) * norDenominator;
            oldMotorTorque = motorTorque;
            motorTorque += newMotorTorque;
            if (motorTorque > stepMotorTorque)                 motorTorque = stepMotorTorque
            // clamp motor torque
            else if (motorTorque < -stepMotorTorque)                 motorTorque = -stepMotorTorque;
            newMotorTorque = motorTorque - oldMotorTorque;
            tmp -= newMotorTorque * invNorDenominator;
        }
        else newMotorTorque = 0;
        
        if (limitSign != 0) {
            newLimitTorque = (tmp - targetAngVelNor) * norDenominator;
            oldLimitTorque = limitTorque;
            limitTorque += newLimitTorque;
            if (limitTorque * limitSign < 0)                 limitTorque = 0  // clamp limit torque  ;
            newLimitTorque = limitTorque - oldLimitTorque;
        }
        else newLimitTorque = 0;
        
        tmp = newMotorTorque + newLimitTorque;
        newTorqueTan = (relVelX * tanX + relVelY * tanY + relVelZ * tanZ - targetAngVelTan) * tanDenominator;
        newTorqueBin = (relVelX * binX + relVelY * binY + relVelZ * binZ - targetAngVelBin) * binDenominator;
        torqueTan += newTorqueTan;
        torqueBin += newTorqueBin;
        
        newImpulseX = tmp * norX + newTorqueTan * tanX + newTorqueBin * binX;
        newImpulseY = tmp * norY + newTorqueTan * tanY + newTorqueBin * binY;
        newImpulseZ = tmp * norZ + newTorqueTan * tanZ + newTorqueBin * binZ;
        
        aVel1.x += invI1e00 * newImpulseX + invI1e01 * newImpulseY + invI1e02 * newImpulseZ;
        aVel1.y += invI1e10 * newImpulseX + invI1e11 * newImpulseY + invI1e12 * newImpulseZ;
        aVel1.z += invI1e20 * newImpulseX + invI1e21 * newImpulseY + invI1e22 * newImpulseZ;
        
        aVel2.x -= invI2e00 * newImpulseX + invI2e01 * newImpulseY + invI2e02 * newImpulseZ;
        aVel2.y -= invI2e10 * newImpulseX + invI2e11 * newImpulseY + invI2e12 * newImpulseZ;
        aVel2.z -= invI2e20 * newImpulseX + invI2e21 * newImpulseY + invI2e22 * newImpulseZ;
    }
    
    /**
		 * @inheritDoc
		 */
    override public function postSolve() : Void{
        impulse.x = impulseX;
        impulse.y = impulseY;
        impulse.z = impulseZ;
        torque.x = torqueX;
        torque.y = torqueY;
        torque.z = torqueZ;
    }
}

