
package oimo.physics.constraint;

import openfl.errors.Error;

import oimo.physics.dynamics.RigidBody;
import oimo.physics.dynamics.World;
/**
	 * 剛体の拘束を扱うクラスです。
	 * 剛体間の接触やジョイントは全て拘束として処理されます。
	 * @author saharan
	 */
class Constraint
{
    /**
		 * この拘束の親となるワールドです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var parent : World;
    
    /**
		 * 拘束の対象となる剛体1です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var body1 : RigidBody;
    
    /**
		 * 拘束の対象となる剛体2です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var body2 : RigidBody;
    
    /**
		 * この拘束がシミュレーションアイランドに追加されたかどうかを示します。
		 * この変数は内部でのみ使用されます。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var addedToIsland : Bool;
    
    /**
		 * この拘束がスリープ状態かどうかを示します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
    public var sleeping : Bool;
    
    /**
		 * 新しく Constraint オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
    public function new()
    {
        
    }
    
    /**
		 * 拘束処理の前に一度だけ呼び出すべきメソッドです。
		 * @param	timeStep 時間刻み幅
		 * @param	invTimeStep 時間刻み幅の逆数
		 */
    public function preSolve(timeStep : Float, invTimeStep : Float) : Void{
        throw new Error("preSolve メソッドが継承されていません");
    }
    
    /**
		 * この拘束を処理します。
		 * 通常このメソッドは繰り返し呼び出され、
		 * 拘束の精度は繰り返し回数に左右されます。
		 */
    public function solve() : Void{
        throw new Error("solve メソッドが継承されていません");
    }
    
    /**
		 * 拘束処理の後に一度だけ呼び出すべきメソッドです。
		 */
    public function postSolve() : Void{
        throw new Error("postSolve メソッドが継承されていません");
    }
}

