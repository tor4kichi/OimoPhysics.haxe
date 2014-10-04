
package oimo.physics.collision.shape;


/**
	 * 球体を表す形状です。
	 * @author saharan
	 */
class SphereShape extends Shape
{
    /**
		 * 球体の半径です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var radius : Float;
    
    /**
		 * 新しく SphereShape オブジェクトを作成します。
		 * @param	radius 球体の半径
		 * @param	config 形状の設定
		 */
    public function new(radius : Float, config : ShapeConfig)
    {
        super();
        this.radius = radius;
        position.copy(config.position);
        rotation.copy(config.rotation);
        friction = config.friction;
        restitution = config.restitution;
        mass = 4 / 3 * Math.PI * radius * radius * radius * config.density;
        var inertia : Float = 2 / 5 * radius * radius * mass;
        localInertia.init(
                inertia, 0, 0,
                0, inertia, 0,
                0, 0, inertia
                );
        type = Shape.SHAPE_SPHERE;
    }
    
    /**
		 * @inheritDoc
		 */
    override public function updateProxy() : Void{
        proxy.init(
                position.x - radius, position.x + radius,
                position.y - radius, position.y + radius,
                position.z - radius, position.z + radius
                );
    }
}

