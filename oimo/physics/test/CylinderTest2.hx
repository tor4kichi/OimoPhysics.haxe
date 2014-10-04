
package oimo.physics.test;


import oimo.physics.collision.shape.BoxShape;
import oimo.physics.collision.shape.CylinderShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.ShapeConfig;
import oimo.physics.collision.shape.SphereShape;
import oimo.physics.constraint.joint.DistanceJoint;
import oimo.physics.constraint.joint.Joint;
import oimo.physics.constraint.joint.JointConfig;
import oimo.physics.dynamics.RigidBody;
import oimo.physics.dynamics.World;
import oimo.math.Mat33;
import oimo.math.Quat;
import oimo.math.Vec3;
import oimo.physics.OimoPhysics;
import oimo.physics.util.DebugDraw;
import flash.display.Sprite;
import flash.display.Stage3D;
import flash.events.Event;
import flash.events.KeyboardEvent;
import flash.text.TextField;
import flash.text.TextFormat;
import flash.ui.Keyboard;
import net.hires.debug.Stats;
/**
	 * 動作テスト
	 * @author saharan
	 */
@:meta(SWF(width="640",height="480",frameRate="60"))

class CylinderTest2 extends Sprite
{
    private var s3d : Stage3D;
    private var world : World;
    private var renderer : DebugDraw;
    private var rigid : RigidBody;
    private var count : Int;
    private var tf : TextField;
    private var fps : Float;
    private var l : Bool;
    private var r : Bool;
    private var u : Bool;
    private var d : Bool;
    private var ctr : RigidBody;
    
    public function new()
    {
        super();
        if (stage)             init()
        else addEventListener(Event.ADDED_TO_STAGE, init);
    }
    
    private function init(e : Event = null) : Void{
        removeEventListener(Event.ADDED_TO_STAGE, init);
        
        var debug : Stats = new Stats();
        debug.x = 570;
        addChild(debug);
        tf = new TextField();
        tf.selectable = false;
        tf.defaultTextFormat = new TextFormat("_monospace", 12, 0xffffff);
        tf.x = 0;
        tf.y = 0;
        tf.width = 400;
        tf.height = 400;
        addChild(tf);
        trace(OimoPhysics.DESCRIPTION);
        initWorld();
        fps = 0;
        
        s3d = stage.stage3Ds[0];
        s3d.addEventListener(Event.CONTEXT3D_CREATE, onContext3DCreated);
        s3d.requestContext3D();
        stage.addEventListener(KeyboardEvent.KEY_DOWN, function(e : KeyboardEvent) : Void{
                    var code : Int = e.keyCode;
                    if (code == Keyboard.W) {
                        u = true;
                    }
                    if (code == Keyboard.S) {
                        d = true;
                    }
                    if (code == Keyboard.A) {
                        l = true;
                    }
                    if (code == Keyboard.D) {
                        r = true;
                    }
                    if (code == Keyboard.SPACE) {
                        initWorld();
                    }
                });
        stage.addEventListener(KeyboardEvent.KEY_UP, function(e : KeyboardEvent) : Void{
                    var code : Int = e.keyCode;
                    if (code == Keyboard.W) {
                        u = false;
                    }
                    if (code == Keyboard.S) {
                        d = false;
                    }
                    if (code == Keyboard.A) {
                        l = false;
                    }
                    if (code == Keyboard.D) {
                        r = false;
                    }
                });
        addEventListener(Event.ENTER_FRAME, frame);
    }
    
    private function initWorld() : Void{
        world = new World();
        if (renderer == null)             renderer = new DebugDraw(640, 480);
        renderer.setWorld(world);
        var rb : RigidBody;
        var s : Shape;
        var c : ShapeConfig = new ShapeConfig();
        // c.friction = 1;
        rb = new RigidBody();
        c.position.init(0, -1, 0);
        c.rotation.init();
        s = new BoxShape(32, 2, 32, c);
        //s = new CylinderShape(16, 2, c);
        rb.addShape(s);
        rb.setupMass(RigidBody.BODY_STATIC);
        world.addRigidBody(rb);
        
        for (k in 0...5){
            for (j in 0...5){
                for (i in 0...7){
                    rb = new RigidBody();
                    c.position.init((j - 2) * 1.4, i * 1.05 + 0.5, (k - 2) * 1.4);
                    s = new CylinderShape(0.5 + Math.random() * 0.1 - 0.05, 1 + Math.random() * 0.1 - 0.05, c);
                    rb.addShape(s);
                    rb.orientation.s = Math.random();
                    rb.orientation.x = Math.random() * 0.01 - 0.005;
                    rb.orientation.y = 1;
                    rb.orientation.z = Math.random() * 0.01 - 0.005;
                    rb.orientation.normalize(rb.orientation);
                    rb.setupMass();
                    world.addRigidBody(rb);
                }
            }
        }
        
        c.density = 10;
        c.position.init(0, 2, 6);
        c.rotation.init();
        s = new BoxShape(2, 2, 2, c);
        ctr = new RigidBody();
        ctr.addShape(s);
        ctr.orientation.s = Math.random() - 0.5;
        ctr.orientation.x = Math.random() - 0.5;
        ctr.orientation.y = Math.random() - 0.5;
        ctr.orientation.z = Math.random() - 0.5;
        ctr.orientation.normalize(ctr.orientation);
        ctr.setupMass();
        world.addRigidBody(ctr);
    }
    
    private function onContext3DCreated(e : Event = null) : Void{
        renderer.setContext3D(s3d.context3D);
        renderer.camera(0, 2, 4);
    }
    
    private function frame(e : Event = null) : Void{
        count++;
        var ang : Float = (320 - mouseX) * 0.01 + Math.PI * 0.5;
        renderer.camera(
                ctr.position.x + Math.cos(ang) * 8,
                ctr.position.y + (240 - mouseY) * 0.1,
                ctr.position.z + Math.sin(ang) * 8,
                ctr.position.x - Math.cos(ang) * 2,
                ctr.position.y,
                ctr.position.z - Math.sin(ang) * 2
                );
        if (l) {
            ctr.linearVelocity.x -= Math.cos(ang - Math.PI * 0.5) * 0.8;
            ctr.linearVelocity.z -= Math.sin(ang - Math.PI * 0.5) * 0.8;
        }
        if (r) {
            ctr.linearVelocity.x -= Math.cos(ang + Math.PI * 0.5) * 0.8;
            ctr.linearVelocity.z -= Math.sin(ang + Math.PI * 0.5) * 0.8;
        }
        if (u) {
            ctr.linearVelocity.x -= Math.cos(ang) * 0.8;
            ctr.linearVelocity.z -= Math.sin(ang) * 0.8;
        }
        if (d) {
            ctr.linearVelocity.x += Math.cos(ang) * 0.8;
            ctr.linearVelocity.z += Math.sin(ang) * 0.8;
        }
        world.step();
        fps += (1000 / world.performance.totalTime - fps) * 0.5;
        if (fps > 1000 || fps != fps) {
            fps = 1000;
        }
        tf.text =
                "Rigid Body Count: " + world.numRigidBodies + "\n" +
                "Shape Count: " + world.numShapes + "\n" +
                "Contact Count: " + world.numContacts + "\n" +
                "Island Count: " + world.numIslands + "\n\n" +
                "Broad Phase Time: " + world.performance.broadPhaseTime + "ms\n" +
                "Narrow Phase Time: " + world.performance.narrowPhaseTime + "ms\n" +
                "Solving Time: " + world.performance.solvingTime + "ms\n" +
                "Updating Time: " + world.performance.updatingTime + "ms\n" +
                "Total Time: " + world.performance.totalTime + "ms\n" +
                "Physics FPS: " + fps.toFixed(2) + "\n";
        renderer.render();
        var len : Int = world.numRigidBodies;
        var rbs : Array<RigidBody> = world.rigidBodies;
        for (i in 0...len){
            var r : RigidBody = rbs[i];
            if (r.position.y < -12) {
                r.position.init(Math.random() * 8 - 4, Math.random() * 4 + 8, Math.random() * 8 - 4);
                r.linearVelocity.x *= 0.8;
                r.linearVelocity.z *= 0.8;
            }
        }
    }
}

