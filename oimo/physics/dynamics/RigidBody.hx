
package oimo.physics.dynamics;

import oimo.physics.dynamics.World;
import openfl.errors.Error;

import oimo.physics.collision.shape.Shape;
import oimo.math.Mat33;
import oimo.math.Quat;
import oimo.math.Vec3;
import oimo.physics.constraint.Constraint;
import oimo.physics.constraint.contact.ContactConnection;
import oimo.physics.constraint.joint.Joint;
import oimo.physics.constraint.joint.JointConnection;
/**
	 * 剛体のクラスです。
	 * 剛体は衝突処理用に単数あるいは複数の形状を持ち、
	 * それぞれ個別にパラメーターを設定できます。
	 * @author saharan
	 */
class RigidBody
{
    /**
		 * 動的な剛体を表す剛体の種類です。
		 */
    public static inline var BODY_DYNAMIC : Int = 0x0;
    
    /**
		 * 静的な剛体を表す剛体の種類です。
		 */
    public static inline var BODY_STATIC : Int = 0x1;
    
    /**
		 * 一つの剛体に追加できる形状の最大数です。
		 */
    public static inline var MAX_SHAPES : Int = 64;
    
    /**
		 * 剛体の種類を表します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 剛体の種類を変更する場合は、必ず
		 * setupMass メソッドの引数に設定したい種類を指定してください。
		 */
    public var type : Int;
    
    /**
		 * 重心のワールド座標です。
		 */
    public var position : Vec3;
    
    /**
		 * 姿勢を表すクォータニオンです。
		 */
    public var orientation : Quat;
    
    /**
		 * スリープ直前での重心のワールド座標です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var sleepPosition : Vec3;
    
    /**
		 * スリープ直前での姿勢を表すクォータニオンです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var sleepOrientation : Quat;
    
    /**
		 * 並進速度です。
		 */
    public var linearVelocity : Vec3;
    
    /**
		 * 角速度です。
		 */
    public var angularVelocity : Vec3;
    
    /**
		 * 姿勢を表す回転行列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 回転行列は、ステップ毎にクォータニオンから再計算されます。
		 */
    public var rotation : Mat33;
    
    /**
		 * 質量です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 質量は setupMass メソッドを呼び出すと、
		 * 含まれている形状から自動で再計算されます。
		 */
    public var mass : Float;
    
    /**
		 * 質量の逆数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 質量は setupMass メソッドを呼び出すと、
		 * 含まれている形状から自動で再計算されます。
		 */
    public var invertMass : Float;
    
    /**
		 * ワールド系での慣性テンソルの逆行列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * ワールド系での慣性テンソルの逆行列は、ステップ毎に
		 * 姿勢と初期状態の慣性テンソルの逆数から再計算されます。
		 */
    public var invertInertia : Mat33;
    
    /**
		 * 初期状態での慣性テンソルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 慣性テンソルは setupMass メソッドを呼び出すと、
		 * 含まれている形状から自動で再計算されます。
		 */
    public var localInertia : Mat33;
    
    /**
		 * 初期状態での慣性テンソルの逆行列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 慣性テンソルは setupMass メソッドを呼び出すと、
		 * 含まれている形状から自動で再計算されます。
		 */
    public var invertLocalInertia : Mat33;
    
    /**
		 * 剛体に含まれている形状の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var shapes : Array<Shape> = new Array<Shape>();
    
    /**
		 * 剛体に含まれている形状の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numShapes : Int;
    
    /**
		 * 剛体が追加されているワールドです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var parent : World;
    
    /**
		 * 剛体に関与する接触点のリンク配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var contactList : ContactConnection;
    
    /**
		 * 剛体に接続されているジョイントのリンク配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var jointList : JointConnection;
    
    /**
		 * 剛体に接続されているジョイントの数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numJoints : Int;
    
    /**
		 * 剛体がシミュレーションアイランドに追加されたかどうかを示します。
		 * この変数は内部でのみ使用されます。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var addedToIsland : Bool;
    
    /**
		 * 剛体が静止してからの時間です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var sleepTime : Float;
    
    /**
		 * 剛体がスリープ状態であるかどうかを示します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 剛体をスリープさせる場合は sleep メソッドを、
		 * 剛体のスリープ状態を解除する場合は awake メソッドを呼び出してください。
		 */
    public var sleeping : Bool;
    
    /**
		 * 剛体をスリープさせるかを示します。
		 * シミュレーションアイランド内の全ての剛体が静止している状態が一定時間続くと、
		 * そのシミュレーションアイランドはスリープ状態に入ります。
		 * スリープしている剛体は awake メソッドが呼び出されるか、
		 * 外部からの干渉を受けるまで、スリープ状態が解除されることはありません。
		 */
    public var allowSleep : Bool;
    
    /**
		 * 新しく RigidBody オブジェクトを作成します。
		 * 回転成分を指定することもできます。
		 * @param	rad ラジアンでの回転角度
		 * @param	ax 回転軸の x 成分
		 * @param	ay 回転軸の y 成分
		 * @param	az 回転軸の z 成分
		 */
    public function new(rad : Float = 0, ax : Float = 0, ay : Float = 0, az : Float = 0)
    {
        position = new Vec3();
        var len : Float = ax * ax + ay * ay + az * az;
        if (len > 0) {
            len = 1 / Math.sqrt(len);
            ax *= len;
            ay *= len;
            az *= len;
        }
        var sin : Float = Math.sin(rad * 0.5);
        var cos : Float = Math.cos(rad * 0.5);
        orientation = new Quat(cos, sin * ax, sin * ay, sin * az);
        linearVelocity = new Vec3();
        angularVelocity = new Vec3();
        sleepPosition = new Vec3();
        sleepOrientation = new Quat();
        rotation = new Mat33();
        invertInertia = new Mat33();
        localInertia = new Mat33();
        invertLocalInertia = new Mat33();
        allowSleep = true;
        sleepTime = 0;
    }
    
    /**
		 * 剛体に形状を追加します。
		 * 形状を追加した場合、次のステップ開始までに setupMass メソッドを呼び出してください。
		 * @param	shape 追加する形状
		 */
    public function addShape(shape : Shape) : Void{
        if (numShapes == MAX_SHAPES) {
            throw new Error("これ以上剛体に形状を追加することはできません");
        }
        if (shape.parent != null) {
            throw new Error("一つの形状を複数剛体に追加することはできません");
        }
        shapes[numShapes++] = shape;
        shape.parent = this;
        if (parent != null)             parent.addShape(shape);
    }
    
    /**
		 * 剛体から形状を削除します。
		 * 削除する形状のインデックスを指定した場合は、インデックスのみを使用して削除します。
		 * 形状を削除した場合、次のステップ開始までに setupMass メソッドを呼び出してください。
		 * @param	shape 削除する形状
		 * @param	index 削除する形状のインデックス
		 */
    public function removeShape(shape : Shape, index : Int = -1) : Void{
        if (index < 0) {
            for (i in 0...numShapes){
                if (shape == shapes[i]) {
                    index = i;
                    break;
                }
            }
            if (index == -1) {
                return;
            }
        }
        else if (index >= numShapes) {
            throw new Error("削除する形状のインデックスが範囲外です");
        }
        var remove : Shape = shapes[index];
        remove.parent = null;
        if (parent != null)             parent.removeShape(remove);
        numShapes--;
        for (j in index...numShapes){
            shapes[j] = shapes[j + 1];
        }
        shapes[numShapes] = null;
    }
    
    /**
		 * 重心座標・質量・慣性テンソルなどを算出します。
		 * type に BODY_STATIC を指定すると、この剛体は空間に対して固定されます。
		 * @param	type 剛体の種類
		 */
    public function setupMass(type : Int = BODY_DYNAMIC) : Void{
        this.type = type;
        position.init();
        mass = 0;
        localInertia.init(0, 0, 0, 0, 0, 0, 0, 0, 0);
        var invRot : Mat33 = new Mat33();  // = rotation ^ -1  
        invRot.transpose(rotation);
        var tmpM : Mat33 = new Mat33();
        var tmpV : Vec3 = new Vec3();
        var denom : Float = 0;
        for (i in 0...numShapes){
            var shape : Shape = shapes[i];
            // relativeRotation = (rotation ^ -1) * shape.rotation
            shape.relativeRotation.mul(invRot, shape.rotation);
            position.add(position, tmpV.scale(shape.position, shape.mass));
            denom += shape.mass;
            mass += shape.mass;
            // inertia = rotation * localInertia * (rotation ^ -1)
            tmpM.mul(shape.relativeRotation, tmpM.mul(shape.localInertia, tmpM.transpose(shape.relativeRotation)));
            localInertia.add(localInertia, tmpM);
        }
        position.scale(position, 1 / denom);
        invertMass = 1 / mass;
        var xy : Float = 0;
        var yz : Float = 0;
        var zx : Float = 0;
        for (j in 0...numShapes){
            var shape : Shape = shapes[j];
            var relPos : Vec3 = shape.localRelativePosition;
            relPos.sub(shape.position, position).mulMat(invRot, relPos);
            shape.updateProxy();
            localInertia.e00 += shape.mass * (relPos.y * relPos.y + relPos.z * relPos.z);
            localInertia.e11 += shape.mass * (relPos.x * relPos.x + relPos.z * relPos.z);
            localInertia.e22 += shape.mass * (relPos.x * relPos.x + relPos.y * relPos.y);
            xy -= shape.mass * relPos.x * relPos.y;
            yz -= shape.mass * relPos.y * relPos.z;
            zx -= shape.mass * relPos.z * relPos.x;
        }
        localInertia.e01 = xy;
        localInertia.e10 = xy;
        localInertia.e02 = zx;
        localInertia.e20 = zx;
        localInertia.e12 = yz;
        localInertia.e21 = yz;
        invertLocalInertia.invert(localInertia);
        if (type == BODY_STATIC) {
            invertMass = 0;
            invertLocalInertia.init(0, 0, 0, 0, 0, 0, 0, 0, 0);
        }
        var r00 : Float = rotation.e00;
        var r01 : Float = rotation.e01;
        var r02 : Float = rotation.e02;
        var r10 : Float = rotation.e10;
        var r11 : Float = rotation.e11;
        var r12 : Float = rotation.e12;
        var r20 : Float = rotation.e20;
        var r21 : Float = rotation.e21;
        var r22 : Float = rotation.e22;
        var i00 : Float = invertLocalInertia.e00;
        var i01 : Float = invertLocalInertia.e01;
        var i02 : Float = invertLocalInertia.e02;
        var i10 : Float = invertLocalInertia.e10;
        var i11 : Float = invertLocalInertia.e11;
        var i12 : Float = invertLocalInertia.e12;
        var i20 : Float = invertLocalInertia.e20;
        var i21 : Float = invertLocalInertia.e21;
        var i22 : Float = invertLocalInertia.e22;
        var e00 : Float = r00 * i00 + r01 * i10 + r02 * i20;
        var e01 : Float = r00 * i01 + r01 * i11 + r02 * i21;
        var e02 : Float = r00 * i02 + r01 * i12 + r02 * i22;
        var e10 : Float = r10 * i00 + r11 * i10 + r12 * i20;
        var e11 : Float = r10 * i01 + r11 * i11 + r12 * i21;
        var e12 : Float = r10 * i02 + r11 * i12 + r12 * i22;
        var e20 : Float = r20 * i00 + r21 * i10 + r22 * i20;
        var e21 : Float = r20 * i01 + r21 * i11 + r22 * i21;
        var e22 : Float = r20 * i02 + r21 * i12 + r22 * i22;
        invertInertia.e00 = e00 * r00 + e01 * r01 + e02 * r02;
        invertInertia.e01 = e00 * r10 + e01 * r11 + e02 * r12;
        invertInertia.e02 = e00 * r20 + e01 * r21 + e02 * r22;
        invertInertia.e10 = e10 * r00 + e11 * r01 + e12 * r02;
        invertInertia.e11 = e10 * r10 + e11 * r11 + e12 * r12;
        invertInertia.e12 = e10 * r20 + e11 * r21 + e12 * r22;
        invertInertia.e20 = e20 * r00 + e21 * r01 + e22 * r02;
        invertInertia.e21 = e20 * r10 + e21 * r11 + e22 * r12;
        invertInertia.e22 = e20 * r20 + e21 * r21 + e22 * r22;
        syncShapes();
        awake();
    }
    
    /**
		 * この剛体をスリープ状態から開放します。
		 */
    public function awake() : Void{
        if (!allowSleep)             return;
        sleepTime = 0;
        if (sleeping) {
            // awake connected constraints
            var cc : ContactConnection = contactList;
            while (cc != null){
                cc.connectedBody.sleepTime = 0;
                cc.connectedBody.sleeping = false;
                cc.parent.sleeping = false;
                cc = cc.next;
            }
            var jc : JointConnection = jointList;
            while (jc != null){
                jc.connected.sleepTime = 0;
                jc.connected.sleeping = false;
                jc.parent.sleeping = false;
                jc = jc.next;
            }
        }
        sleeping = false;
    }
    
    /**
		 * この剛体をスリープさせます。
		 */
    public function sleep() : Void{
        if (!allowSleep)             return;
        linearVelocity.init();
        angularVelocity.init();
        sleepPosition.copy(position);
        sleepOrientation.copy(orientation);
        sleepTime = 0;
        sleeping = true;
    }
    
    /**
		 * 剛体の外力による速度変化を計算します。
		 * このメソッドはワールドのステップを呼ぶと自動で呼び出されるので、
		 * 通常は外部から呼ぶ必要はありません。
		 * @param	timeStep 時間刻み幅
		 * @param	gravity 重力
		 */
    public function updateVelocity(timeStep : Float, gravity : Vec3) : Void{
        if (type == BODY_DYNAMIC) {
            linearVelocity.x += gravity.x * timeStep;
            linearVelocity.y += gravity.y * timeStep;
            linearVelocity.z += gravity.z * timeStep;
        }
    }
    
    /**
		 * 剛体の運動を時間積分し、形状などの情報を更新します。
		 * このメソッドはワールドのステップを呼ぶと自動で呼び出されるので、
		 * 通常は外部から呼ぶ必要はありません。
		 * @param	timeStep 時間刻み幅
		 */
    public function updatePosition(timeStep : Float) : Void{
        if (!allowSleep)             sleepTime = 0;
        if (type == BODY_STATIC) {
            linearVelocity.x = 0;
            linearVelocity.y = 0;
            linearVelocity.z = 0;
            angularVelocity.x = 0;
            angularVelocity.y = 0;
            angularVelocity.z = 0;
        }
        else if (type == BODY_DYNAMIC) {
            var vx : Float = linearVelocity.x;
            var vy : Float = linearVelocity.y;
            var vz : Float = linearVelocity.z;
            if (vx * vx + vy * vy + vz * vz > 0.01)                 sleepTime = 0;
            position.x += vx * timeStep;
            position.y += vy * timeStep;
            position.z += vz * timeStep;
            vx = angularVelocity.x;
            vy = angularVelocity.y;
            vz = angularVelocity.z;
            if (vx * vx + vy * vy + vz * vz > 0.025)                 sleepTime = 0;
            var os : Float = orientation.s;
            var ox : Float = orientation.x;
            var oy : Float = orientation.y;
            var oz : Float = orientation.z;
            timeStep *= 0.5;
            var s : Float = (-vx * ox - vy * oy - vz * oz) * timeStep;
            var x : Float = (vx * os + vy * oz - vz * oy) * timeStep;
            var y : Float = (-vx * oz + vy * os + vz * ox) * timeStep;
            var z : Float = (vx * oy - vy * ox + vz * os) * timeStep;
            os += s;
            ox += x;
            oy += y;
            oz += z;
            s = 1 / Math.sqrt(os * os + ox * ox + oy * oy + oz * oz);
            orientation.s = os * s;
            orientation.x = ox * s;
            orientation.y = oy * s;
            orientation.z = oz * s;
        }
        else {
            throw new Error("未定義の剛体の種類です");
        }
        syncShapes();
    }
    
    private function syncShapes() : Void{
        var s : Float = orientation.s;
        var x : Float = orientation.x;
        var y : Float = orientation.y;
        var z : Float = orientation.z;
        var x2 : Float = 2 * x;
        var y2 : Float = 2 * y;
        var z2 : Float = 2 * z;
        var xx : Float = x * x2;
        var yy : Float = y * y2;
        var zz : Float = z * z2;
        var xy : Float = x * y2;
        var yz : Float = y * z2;
        var xz : Float = x * z2;
        var sx : Float = s * x2;
        var sy : Float = s * y2;
        var sz : Float = s * z2;
        rotation.e00 = 1 - yy - zz;
        rotation.e01 = xy - sz;
        rotation.e02 = xz + sy;
        rotation.e10 = xy + sz;
        rotation.e11 = 1 - xx - zz;
        rotation.e12 = yz - sx;
        rotation.e20 = xz - sy;
        rotation.e21 = yz + sx;
        rotation.e22 = 1 - xx - yy;
        var r00 : Float = rotation.e00;
        var r01 : Float = rotation.e01;
        var r02 : Float = rotation.e02;
        var r10 : Float = rotation.e10;
        var r11 : Float = rotation.e11;
        var r12 : Float = rotation.e12;
        var r20 : Float = rotation.e20;
        var r21 : Float = rotation.e21;
        var r22 : Float = rotation.e22;
        var i00 : Float = invertLocalInertia.e00;
        var i01 : Float = invertLocalInertia.e01;
        var i02 : Float = invertLocalInertia.e02;
        var i10 : Float = invertLocalInertia.e10;
        var i11 : Float = invertLocalInertia.e11;
        var i12 : Float = invertLocalInertia.e12;
        var i20 : Float = invertLocalInertia.e20;
        var i21 : Float = invertLocalInertia.e21;
        var i22 : Float = invertLocalInertia.e22;
        var e00 : Float = r00 * i00 + r01 * i10 + r02 * i20;
        var e01 : Float = r00 * i01 + r01 * i11 + r02 * i21;
        var e02 : Float = r00 * i02 + r01 * i12 + r02 * i22;
        var e10 : Float = r10 * i00 + r11 * i10 + r12 * i20;
        var e11 : Float = r10 * i01 + r11 * i11 + r12 * i21;
        var e12 : Float = r10 * i02 + r11 * i12 + r12 * i22;
        var e20 : Float = r20 * i00 + r21 * i10 + r22 * i20;
        var e21 : Float = r20 * i01 + r21 * i11 + r22 * i21;
        var e22 : Float = r20 * i02 + r21 * i12 + r22 * i22;
        invertInertia.e00 = e00 * r00 + e01 * r01 + e02 * r02;
        invertInertia.e01 = e00 * r10 + e01 * r11 + e02 * r12;
        invertInertia.e02 = e00 * r20 + e01 * r21 + e02 * r22;
        invertInertia.e10 = e10 * r00 + e11 * r01 + e12 * r02;
        invertInertia.e11 = e10 * r10 + e11 * r11 + e12 * r12;
        invertInertia.e12 = e10 * r20 + e11 * r21 + e12 * r22;
        invertInertia.e20 = e20 * r00 + e21 * r01 + e22 * r02;
        invertInertia.e21 = e20 * r10 + e21 * r11 + e22 * r12;
        invertInertia.e22 = e20 * r20 + e21 * r21 + e22 * r22;
        for (i in 0...numShapes){
            var shape : Shape = shapes[i];
            var relPos : Vec3 = shape.relativePosition;
            var lRelPos : Vec3 = shape.localRelativePosition;
            var relRot : Mat33 = shape.relativeRotation;
            var rot : Mat33 = shape.rotation;
            var lx : Float = lRelPos.x;
            var ly : Float = lRelPos.y;
            var lz : Float = lRelPos.z;
            relPos.x = lx * r00 + ly * r01 + lz * r02;
            relPos.y = lx * r10 + ly * r11 + lz * r12;
            relPos.z = lx * r20 + ly * r21 + lz * r22;
            shape.position.x = position.x + relPos.x;
            shape.position.y = position.y + relPos.y;
            shape.position.z = position.z + relPos.z;
            e00 = relRot.e00;
            e01 = relRot.e01;
            e02 = relRot.e02;
            e10 = relRot.e10;
            e11 = relRot.e11;
            e12 = relRot.e12;
            e20 = relRot.e20;
            e21 = relRot.e21;
            e22 = relRot.e22;
            rot.e00 = r00 * e00 + r01 * e10 + r02 * e20;
            rot.e01 = r00 * e01 + r01 * e11 + r02 * e21;
            rot.e02 = r00 * e02 + r01 * e12 + r02 * e22;
            rot.e10 = r10 * e00 + r11 * e10 + r12 * e20;
            rot.e11 = r10 * e01 + r11 * e11 + r12 * e21;
            rot.e12 = r10 * e02 + r11 * e12 + r12 * e22;
            rot.e20 = r20 * e00 + r21 * e10 + r22 * e20;
            rot.e21 = r20 * e01 + r21 * e11 + r22 * e21;
            rot.e22 = r20 * e02 + r21 * e12 + r22 * e22;
            shape.updateProxy();
        }
    }
    
    public function applyImpulse(position : Vec3, force : Vec3) : Void{
        linearVelocity.x += force.x * invertMass;
        linearVelocity.y += force.y * invertMass;
        linearVelocity.z += force.z * invertMass;
        var rel : Vec3 = new Vec3();
        rel.sub(position, this.position).cross(rel, force).mulMat(invertInertia, rel);
        angularVelocity.x += rel.x;
        angularVelocity.y += rel.y;
        angularVelocity.z += rel.z;
    }
}

