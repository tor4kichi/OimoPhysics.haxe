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
package oimo.physics.dynamics;

import openfl.errors.Error;

import oimo.math.Quat;
import oimo.physics.collision.broad.BroadPhase;
import oimo.physics.collision.broad.BruteForceBroadPhase;
import oimo.physics.collision.broad.Pair;
import oimo.physics.collision.broad.SweepAndPruneBroadPhase;
import oimo.physics.collision.narrow.BoxBoxCollisionDetector;
import oimo.physics.collision.narrow.BoxCylinderCollisionDetector;
import oimo.physics.collision.narrow.CollisionResult;
import oimo.physics.collision.narrow.ContactInfo;
import oimo.physics.collision.narrow.CollisionDetector;
import oimo.physics.collision.narrow.CylinderCylinderCollisionDetector;
import oimo.physics.collision.narrow.SphereBoxCollisionDetector;
import oimo.physics.collision.narrow.SphereCylinderCollisionDetector;
import oimo.physics.collision.narrow.SphereSphereCollisionDetector;
import oimo.physics.collision.shape.Shape;
import oimo.math.Vec3;
import oimo.physics.constraint.Constraint;
import oimo.physics.constraint.contact.Contact;
import oimo.physics.constraint.contact.ContactConnection;
import oimo.physics.constraint.joint.Joint;
import oimo.physics.constraint.joint.JointConnection;
import oimo.physics.util.Performance;

/**
	 * 物理演算ワールドのクラスです。
	 * 全ての物理演算オブジェクトはワールドに追加する必要があります。
	 * @author saharan
	 */
class World
{
    /**
		 * 追加できる剛体の最大数です。
		 */
    public static inline var MAX_BODIES : Int = 16384;
    
    /**
		 * 追加できる形状の最大数です。
		 */
    public static inline var MAX_SHAPES : Int = 32768;
    
    /**
		 * 検出できる接触点の最大数です。
		 */
    public static inline var MAX_CONTACTS : Int = 65536;
    
    /**
		 * 追加できるジョイントの最大数です。
		 */
    public static inline var MAX_JOINTS : Int = 16384;
    
    private static var MAX_CONSTRAINTS : Int = MAX_CONTACTS + MAX_JOINTS;
    
    /**
		 * 追加されている剛体の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var rigidBodies : Array<RigidBody> = new Array<RigidBody>();
    
    /**
		 * 追加されている剛体の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numRigidBodies : Int;
    
    /**
		 * 追加されている形状の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var shapes : Array<Shape> = new Array<Shape>();
    
    /**
		 * 追加されている形状の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numShapes : Int;
    
    /**
		 * 剛体の接触点の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var contacts : Array<Contact>;
    private var prevContacts : Array<Contact>;
    private var contactPool1 : Array<Contact> = new Array<Contact>();
    private var contactPool2 : Array<Contact> = new Array<Contact>();
    
    /**
		 * 剛体の接触点の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numContacts : Int;
    public var numPrevContacts1 : Int;
    public var numPrevContacts2 : Int;
    
    /**
		 * ジョイントの配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var joints : Array<Joint> = new Array<Joint>();
    
    /**
		 * ジョイントの数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numJoints : Int;
    
    /**
		 * シミュレーションアイランドの数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var numIslands : Int;
    
    private var constraints : Array<Constraint> = new Array<Constraint>();
    private var numConstraints : Int;
    
    /**
		 * 1回のステップで進む時間の長さです。
		 */
    public var timeStep : Float;
    
    /**
		 * ワールドにかかる重力です。
		 */
    public var gravity : Vec3;
    
    /**
		 * 衝突応答の反復処理回数です。
		 * 値が大きいほど、より正確な動きになります。
		 */
    public var iteration : Int;
    
    /**
		 * パフォーマンスの詳細情報です。
		 * 計算に要した時間などが記録されています。
		 */
    public var performance : Performance;
    
    /**
		 * 詳細な衝突判定をできるだけ削減するために使用される広域衝突判定です。
		 */
    public var broadPhase : BroadPhase;
    
    private var detectors : Array<Array<CollisionDetector>>;
    private var collisionResult : CollisionResult = new CollisionResult(MAX_CONTACTS);
    
    private var islandStack : Array<RigidBody> = new Array<RigidBody>();
    private var islandRigidBodies : Array<RigidBody> = new Array<RigidBody>();
    private var islandNumRigidBodies : Int;
    private var islandConstraints : Array<Constraint> = new Array<Constraint>();
    private var islandNumConstraints : Int;
    
    private var randX : Int;
    private var randA : Int;
    private var randB : Int;
    
    /**
		 * 新しく World オブジェクトを作成します。
		 * ワールドのタイムステップは、1秒間でのステップの実行回数から算出されます。
		 * @param stepPerSecond 1秒間でのステップの実行回数
		 */
    public function new(stepPerSecond : Float = 60)
    {
        timeStep = 1 / stepPerSecond;
        iteration = 8;
        gravity = new Vec3(0, -9.80665, 0);
        performance = new Performance();
        broadPhase = new SweepAndPruneBroadPhase();
        // broadPhase = new BruteForceBroadPhase();
        var numShapeTypes : Int = 4;
        detectors = new Array<Array<CollisionDetector>>();
        for (i in 0...numShapeTypes){
            detectors[i] = new Array<CollisionDetector>();
        }
        detectors[Shape.SHAPE_SPHERE][Shape.SHAPE_SPHERE] = new SphereSphereCollisionDetector();
        detectors[Shape.SHAPE_SPHERE][Shape.SHAPE_BOX] = new SphereBoxCollisionDetector(false);
        detectors[Shape.SHAPE_SPHERE][Shape.SHAPE_CYLINDER] = new SphereCylinderCollisionDetector(false);
        detectors[Shape.SHAPE_BOX][Shape.SHAPE_SPHERE] = new SphereBoxCollisionDetector(true);
        detectors[Shape.SHAPE_BOX][Shape.SHAPE_BOX] = new BoxBoxCollisionDetector();
        detectors[Shape.SHAPE_CYLINDER][Shape.SHAPE_SPHERE] = new SphereCylinderCollisionDetector(true);
        detectors[Shape.SHAPE_BOX][Shape.SHAPE_CYLINDER] = new BoxCylinderCollisionDetector(false);
        detectors[Shape.SHAPE_CYLINDER][Shape.SHAPE_BOX] = new BoxCylinderCollisionDetector(true);
        detectors[Shape.SHAPE_CYLINDER][Shape.SHAPE_CYLINDER] = new CylinderCylinderCollisionDetector();
        contacts = contactPool1;
        prevContacts = contactPool2;
        randX = 65535;
        randA = 98765;
        randB = 123456789;
    }
    
    /**
		 * ワールドに剛体を追加します。
		 * 追加された剛体はステップ毎の演算対象になります。
		 * @param	rigidBody 追加する剛体
		 */
    public function addRigidBody(rigidBody : RigidBody) : Void{
        if (numRigidBodies == MAX_BODIES) {
            throw new Error("これ以上ワールドに剛体を追加することはできません");
        }
        if (rigidBody.parent != null) {
            throw new Error("一つの剛体を複数ワールドに追加することはできません");
        }
        rigidBody.awake();
        var num : Int = rigidBody.numShapes;
        for (i in 0...num){
            addShape(rigidBody.shapes[i]);
        }
        rigidBodies[numRigidBodies++] = rigidBody;
        rigidBody.parent = this;
    }
    
    /**
		 * ワールドから剛体を削除します。
		 * 削除された剛体はステップ毎の演算対象から外されます。
		 * @param	rigidBody 削除する剛体
		 */
    public function removeRigidBody(rigidBody : RigidBody) : Void{
        var remove : RigidBody = null;
        for (i in 0...numRigidBodies){
            if (rigidBodies[i] == rigidBody) {
                remove = rigidBody;
                rigidBodies[i] = rigidBodies[--numRigidBodies];
                rigidBodies[numRigidBodies] = null;
                break;
            }
        }
        if (remove == null)             return;
        remove.awake();
        var jc : JointConnection = remove.jointList;
        while (jc != null){
            var joint : Joint = jc.parent;
            jc = jc.next;
            removeJoint(joint);
        }
        var num : Int = remove.numShapes;
        for (i in 0...num){
            removeShape(remove.shapes[i]);
        }
        remove.parent = null;
    }
    
    /**
		 * ワールドに形状を追加します。
		 * <strong>剛体をワールドに追加、およびワールドに追加されている剛体に形状を追加すると、
		 * 自動で形状もワールドに追加されるので、このメソッドは外部から呼ばないでください。</strong>
		 * @param	shape 追加する形状
		 */
    public function addShape(shape : Shape) : Void{
        if (numShapes == MAX_SHAPES) {
            throw new Error("これ以上ワールドに形状を追加することはできません");
        }
        if (shape.parent == null) {
            throw new Error("ワールドに形状を単体で追加することはできません");
        }
        if (shape.parent.parent != null) {
            throw new Error("一つの形状を複数ワールドに追加することはできません");
        }
        broadPhase.addProxy(shape.proxy);
        shapes[numShapes++] = shape;
    }
    
    /**
		 * ワールドから形状を削除します。
		 * <strong>剛体をワールドから削除、およびワールドに追加されている剛体から形状を削除すると、
		 * 自動で形状もワールドから削除されるので、このメソッドは外部から呼ばないでください。</strong>
		 * @param	shape 削除する形状
		 */
    public function removeShape(shape : Shape) : Void{
        var remove : Shape = null;
        for (i in 0...numShapes){
            if (shapes[i] == shape) {
                remove = shape;
                shapes[i] = shapes[--numShapes];
                shapes[numShapes] = null;
                break;
            }
        }
        if (remove == null)             return;
        broadPhase.removeProxy(remove.proxy);
    }
    
    /**
		 * ワールドにジョイントを追加します。
		 * 追加されたジョイントはステップ毎の演算対象になります。
		 * @param	joint 追加するジョイント
		 */
    public function addJoint(joint : Joint) : Void{
        if (numJoints == MAX_JOINTS) {
            throw new Error("これ以上ワールドにジョイントを追加することはできません");
        }
        if (joint.parent != null) {
            throw new Error("一つのジョイントを複数ワールドに追加することはできません");
        }
        var b : RigidBody = joint.body1;
        var jc : JointConnection = joint.connection1;
        b.awake();
        b.numJoints++;
        jc.next = b.jointList;
        if (b.jointList != null) {
            b.jointList.prev = jc;
        }
        b.jointList = jc;
        b = joint.body2;
        jc = joint.connection2;
        b.awake();
        b.numJoints++;
        jc.next = b.jointList;
        if (b.jointList != null) {
            b.jointList.prev = jc;
        }
        b.jointList = jc;
        joints[numJoints++] = joint;
        joint.parent = this;
    }
    
    /**
		 * ワールドからジョイントを削除します。
		 * 削除されたジョイントはステップ毎の演算対象から外されます。
		 * @param	joint 削除するジョイント
		 * @param	index 削除するジョイントのインデックス
		 */
    public function removeJoint(joint : Joint) : Void{
        var remove : Joint = null;
        for (i in 0...numJoints){
            if (joints[i] == joint) {
                remove = joint;
                joints[i] = joints[--numJoints];
                joints[numJoints] = null;
                break;
            }
        }
        if (remove == null)             return;
        remove.body1.awake();
        remove.body1.numJoints--;
        remove.body2.awake();
        remove.body2.numJoints--;
        var jc : JointConnection = remove.connection1;
        if (jc.prev != null) {
            jc.prev.next = jc.next;
            jc.prev = null;
        }
        if (jc.next != null) {
            jc.next.prev = jc.prev;
            jc.next = null;
        }
        jc = remove.connection2;
        if (jc.prev != null) {
            jc.prev.next = jc.next;
            jc.prev = null;
        }
        if (jc.next != null) {
            jc.next.prev = jc.prev;
            jc.next = null;
        }
        remove.parent = null;
    }
    
    /**
		 * ワールドの時間をタイムステップ秒だけ進めます。
		 */
    public function step() : Void{
        var time1 : Int = Math.round(haxe.Timer.stamp() * 1000);
        var tmpC : Array<Contact> = contacts;  // swap contacts  
        contacts = prevContacts;
        prevContacts = tmpC;
        for (i in 0...numRigidBodies){
            var tmpB : RigidBody = rigidBodies[i];
            if (tmpB.sleeping) {
                var lv : Vec3 = tmpB.linearVelocity;
                var av : Vec3 = tmpB.linearVelocity;
                var p : Vec3 = tmpB.position;
                var sp : Vec3 = tmpB.sleepPosition;
                var o : Quat = tmpB.orientation;
                var so : Quat = tmpB.sleepOrientation;
                if (
                    lv.x != 0 || lv.y != 0 || lv.z != 0 ||
                    av.x != 0 || av.y != 0 || av.z != 0 ||
                    p.x != sp.x || p.y != sp.y || p.z != sp.z ||
                    o.s != so.s || o.x != so.x || o.y != so.y || o.z != so.z) {
                    tmpB.awake();  // awaking check  
                    {
						continue;
                    }
                }
            }
        }
        detectCollisions();
        updateIslands();
        var time2 : Int = Math.round(haxe.Timer.stamp() * 1000);
        performance.solvingTime = time2 - performance.solvingTime;
        performance.totalTime = time2 - time1;
    }
    
    private function detectCollisions() : Void{
        collectContactInfos();
        setupContacts();
    }
    
    private function collectContactInfos() : Void{
        // broad phase
        var time1 : Int = Math.round(haxe.Timer.stamp() * 1000);
        broadPhase.detectPairs();
        var time2 : Int = Math.round(haxe.Timer.stamp() * 1000);
        performance.broadPhaseTime = time2 - time1;
        // narrow phase
        collisionResult.numContactInfos = 0;
        var pairs : Array<Pair> = broadPhase.pairs;
        var numPairs : Int = broadPhase.numPairs;
        for (i in 0...numPairs){
            var pair : Pair = pairs[i];
            var s1 : Shape = pair.shape1;
            var s2 : Shape = pair.shape2;
            var detector : CollisionDetector = detectors[s1.type][s2.type];
            if (detector != null) {
                detector.detectCollision(s1, s2, collisionResult);
                if (collisionResult.numContactInfos == MAX_CONTACTS) {
                    return;
                }
            }
        }
        var time3 : Int = Math.round(haxe.Timer.stamp() * 1000);
        performance.narrowPhaseTime = time3 - time2;
        performance.updatingTime = time3;
    }
    
    private function setupContacts() : Void{
        numPrevContacts2 = numPrevContacts1;
        numPrevContacts1 = numContacts;
        var numSleptContacts : Int = 0;
        for (i in 0...numPrevContacts1){
            var c : Contact = prevContacts[i];
            if (c.sleeping) {  // keep slept contacts  
                prevContacts[i] = contacts[numSleptContacts];
                contacts[numSleptContacts++] = c;
            }
        }
        var contactInfos : Array<ContactInfo> = collisionResult.contactInfos;
        numContacts = numSleptContacts + collisionResult.numContactInfos;
        for (i in numSleptContacts...numContacts){
            if (contacts[i] == null) {
                contacts[i] = new Contact();
            }
            var c : Contact = contacts[i];
            c.setupFromContactInfo(contactInfos[i - numSleptContacts]);
            // search old contacts
            var s1 : Shape = c.shape1;
            var s2 : Shape = c.shape2;
            var cc : ContactConnection;
            if (s1.numContacts < s2.numContacts)                 cc = s1.contactList
            else cc = s2.contactList;
            while (cc != null){
                var old : Contact = cc.parent;
                if (
                    (old.shape1 == c.shape1 && old.shape2 == c.shape2 ||
                    old.shape1 == c.shape2 && old.shape2 == c.shape1) &&
                    old.id.equals(c.id)) {
                    // warm starting
                    c.normalImpulse = old.normalImpulse;
                    c.tangentImpulse = old.tangentImpulse;
                    c.binormalImpulse = old.binormalImpulse;
                    c.warmStarted = true;
                    break;
                }
                cc = cc.next;
            }
        }
        for (i in numContacts...numPrevContacts2){
            contacts[i].removeReferences();
        }
    }
    
    private function updateIslands() : Void{
        var invTimeStep : Float = 1 / timeStep;
        var tmpC : Constraint;
        var tmpB : RigidBody;
        var tmpS : Shape;
        var num : Int;
        
        // reset contact and connection counts
        for (i in 0...numShapes){
            tmpS = shapes[i];
            tmpS.contactList = null;
            tmpS.numContacts = 0;
        }
        for (i in 0...numRigidBodies){
            tmpB = rigidBodies[i];
            tmpB.contactList = null;
            tmpB.addedToIsland = false;
        }  // collect all constraints  
        
        
        
        numConstraints = 0;
        for (i in 0...numContacts){
            var c : Contact = contacts[i];
            c.addedToIsland = false;
            // add to shape1
            var cc : ContactConnection = c.shapeConnection1;
            tmpS = c.shape1;
            cc.prev = null;
            cc.next = tmpS.contactList;
            if (tmpS.contactList != null) {
                tmpS.contactList.prev = cc;
            }
            tmpS.contactList = cc;
            tmpS.numContacts++;
            // add to shape2
            cc = c.shapeConnection2;
            tmpS = c.shape2;
            cc.prev = null;
            cc.next = tmpS.contactList;
            if (tmpS.contactList != null) {
                tmpS.contactList.prev = cc;
            }
            tmpS.contactList = cc;
            tmpS.numContacts++;
            // add to body1
            cc = c.bodyConnection1;
            tmpB = c.body1;
            cc.prev = null;
            cc.next = tmpB.contactList;
            if (tmpB.contactList != null) {
                tmpB.contactList.prev = cc;
            }
            tmpB.contactList = cc;
            // add to body2
            cc = c.bodyConnection2;
            tmpB = c.body2;
            cc.prev = null;
            cc.next = tmpB.contactList;
            if (tmpB.contactList != null) {
                tmpB.contactList.prev = cc;
            }
            tmpB.contactList = cc;
            constraints[numConstraints++] = c;
        }
        for (i in 0...numJoints){
            tmpC = joints[i];
            tmpC.addedToIsland = false;
            constraints[numConstraints++] = tmpC;
        }  // randomizing order TODO: it should be able to be disabled by simulation setting  
        
        
        
        for (i in 1...numConstraints){
            var swap : Int = Std.int((randX = (randX * randA + randB & 0x7fffffff)) / 2147483648.0 * i);
            tmpC = constraints[i];
            constraints[i] = constraints[swap];
            constraints[swap] = tmpC;
        }
        
        var time1 : Int = Math.round(haxe.Timer.stamp() * 1000);
        performance.updatingTime = time1 - performance.updatingTime;
        performance.solvingTime = time1;
        
        numIslands = 0;
        // build and solve simulation islands
		var rigidBodyItr:Int = 0;
		while(rigidBodyItr < numRigidBodies) {
            var base : RigidBody = rigidBodies[rigidBodyItr];
            if (
				base.addedToIsland || 
				base.type == RigidBody.BODY_STATIC || 
				base.sleeping
				) {
				rigidBodyItr++;
				continue;
            }  // ignore  ;
            islandNumRigidBodies = 0;
            islandNumConstraints = 0;
            var numStacks : Int = 1;
            // add rigid body to stack
            islandStack[0] = base;
            base.addedToIsland = true;
            // build an island
            while (numStacks > 0){
                // get rigid body from stack
                tmpB = islandStack[--numStacks];
                tmpB.sleeping = false;
                // add rigid body to the island
                islandRigidBodies[islandNumRigidBodies++] = tmpB;
                // search connections
                var cc : ContactConnection = tmpB.contactList;
                while (cc != null){
                    tmpC = cc.parent;
                    if (tmpC.addedToIsland) {
                        cc = cc.next;
                        {rigidBodyItr++;continue;
                        }
                    }  // add constraint to the island  
                    
                    islandConstraints[islandNumConstraints++] = tmpC;
                    tmpC.addedToIsland = true;
                    tmpC.sleeping = false;
                    var next : RigidBody = cc.connectedBody;
                    if (next.addedToIsland || next.type == RigidBody.BODY_STATIC) {
                        cc = cc.next;
                        {rigidBodyItr++;continue;
                        }
                    }  // add rigid body to stack  
                    
                    islandStack[numStacks++] = next;
                    next.addedToIsland = true;
                    cc = cc.next;
                }
                var jc : JointConnection = tmpB.jointList;
                while (jc != null){
                    tmpC = jc.parent;
                    if (tmpC.addedToIsland) {
                        jc = jc.next;
                        {rigidBodyItr++;continue;
                        }
                    }  // add constraint to the island  
                    
                    islandConstraints[islandNumConstraints++] = tmpC;
                    tmpC.addedToIsland = true;
                    tmpC.sleeping = false;
                    var next : RigidBody = jc.connected;
                    if (next.addedToIsland || next.type == RigidBody.BODY_STATIC) {
                        jc = jc.next;
                        {rigidBodyItr++;continue;
                        }
                    }  // add rigid body to stack  
                    
                    islandStack[numStacks++] = next;
                    next.addedToIsland = true;
                    jc = jc.next;
                }
            }  // update velocities    // update the island  
            
            
            
            
            for (j in 0...islandNumRigidBodies){
                tmpB = islandRigidBodies[j];
                tmpB.updateVelocity(timeStep, gravity);
            }  // solve contraints  
            
            
            
            for (j in 0...islandNumConstraints){
                islandConstraints[j].preSolve(timeStep, invTimeStep);
            }
            for (j in 0...iteration){
                for (k in 0...islandNumConstraints){
                    islandConstraints[k].solve();
                }
            }
            for (j in 0...islandNumConstraints){
                islandConstraints[j].postSolve();
            }  // sleeping check  
            
            
            
            var sleepTime : Float = 1000;
            for (j in 0...islandNumRigidBodies){
                tmpB = islandRigidBodies[j];
                if (!tmpB.allowSleep) {
                    tmpB.sleepTime = 0;
                    sleepTime = 0;
                    {
						continue;
                    }
                }
                var vx : Float = tmpB.linearVelocity.x;
                var vy : Float = tmpB.linearVelocity.y;
                var vz : Float = tmpB.linearVelocity.z;
                if (vx * vx + vy * vy + vz * vz > 0.01) {
                    tmpB.sleepTime = 0;
                    sleepTime = 0;
                    {
						continue;
                    }
                }
                vx = tmpB.angularVelocity.x;
                vy = tmpB.angularVelocity.y;
                vz = tmpB.angularVelocity.z;
                if (vx * vx + vy * vy + vz * vz > 0.04) {
                    tmpB.sleepTime = 0;
                    sleepTime = 0;
                    {
						continue;
                    }
                }
                tmpB.sleepTime += timeStep;
                if (tmpB.sleepTime < sleepTime)                     sleepTime = tmpB.sleepTime;
            }
            if (sleepTime > 0.5) {
                // sleep the island
                for (j in 0...islandNumRigidBodies){
                    tmpB = islandRigidBodies[j];
                    tmpB.linearVelocity.init();
                    tmpB.angularVelocity.init();
                    tmpB.sleepPosition.copy(tmpB.position);
                    tmpB.sleepOrientation.copy(tmpB.orientation);
                    tmpB.sleepTime = 0;
                    tmpB.sleeping = true;
                }
                for (j in 0...islandNumConstraints){
                    islandConstraints[j].sleeping = true;
                }
            }
            else {
                // update positions
                for (j in 0...islandNumRigidBodies){
                    islandRigidBodies[j].updatePosition(timeStep);
                }
            }
            numIslands++;
        }
		rigidBodyItr++;
    }
}

