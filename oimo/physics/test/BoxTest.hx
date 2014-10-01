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

class BoxTest extends Sprite
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
        tf.x = 4;
        tf.y = 4;
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
        world.iteration = 8;
        if (renderer == null)             renderer = new DebugDraw(640, 480);
        renderer.setWorld(world);
        var rb : RigidBody;
        var s : Shape;
        var c : ShapeConfig = new ShapeConfig();
        // c.friction = 0;
        c.restitution = 0;
        rb = new RigidBody();
        c.position.init(0, -0.5, 0);
        c.rotation.init();
        s = new BoxShape(32, 1, 32, c);
        rb.addShape(s);
        rb.setupMass(RigidBody.BODY_STATIC);
        world.addRigidBody(rb);
        c.rotation.init();
        var width : Int = 6;
        var height : Int = 6;
        var depth : Int = 6;
        var bw : Float = 0.75;
        var bh : Float = 0.75;
        var bd : Float = 0.75;
        for (i in 0...width){
            for (j in 0...height){
                for (k in 0...depth){
                    rb = new RigidBody();
                    c.position.init(
                            (i - (width - 1) * 0.5) * bw,
                            j * (bh * 1.01) + bh * 0.5,
                            (k - (depth - 1) * 0.5) * bd
                            );
                    //if (Math.random() > 0.5) s = new SphereShape((bw + bh + bd) / 6, c); else
                    //if (Math.random() > 0.5) s = new CylinderShape((bw + bd) / 4, bh, c); else
                    s = new BoxShape(bw, bh, bd, c);
                    rb.addShape(s);
                    rb.setupMass(RigidBody.BODY_DYNAMIC);
                    world.addRigidBody(rb);
                }
                Shape.nextID++;
            }
            Shape.nextID++;
        }  /*for (var i:int = 0; i < height; i++) {
				rb = new RigidBody();
				c.position.init(
					0,
					i * (bh * 1.1) + bh * 0.5 + 0.05,
					0
				);
				s = new SphereShape(bh * 0.5, c);
				rb.addShape(s);
				rb.setupMass(RigidBody.BODY_DYNAMIC);
				world.addRigidBody(rb);
			}*/    /*for (var j:int = 0; j < height; j++) {
				for (var i:int = 0; i < 3; i++) {
						var rot:Boolean = (j & 1) == 0;
						rb = new RigidBody();
						c.position.init(
							rot ? 0 : (i - 1) * 0.75,
							j * (0.75 * 1.1) + 0.75 * 0.5 + 0.05,
							rot ? (i - 1) * 0.75 : 0
						);
						s = new BoxShape(rot ? 2.25 : 0.75, 0.75, rot ? 0.75 : 2.25, c);
						rb.addShape(s);
						rb.setupMass(RigidBody.BODY_DYNAMIC);
						world.addRigidBody(rb);
				}
			}*/  
        
        
        
        
        c.friction = 2;
        c.position.init(0, 1, 8);
        c.density = 10;
        c.rotation.init();
        s = new BoxShape(2, 2, 2, c);
        ctr = new RigidBody();
        ctr.addShape(s);
        ctr.setupMass(RigidBody.BODY_DYNAMIC);
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

