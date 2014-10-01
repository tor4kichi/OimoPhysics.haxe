package oimo.physics.util;


/**
	 * ワールドが物理演算に要した時間などを記録するクラスです。
	 * 特に表記がない場合、時間の単位はミリ秒です。
	 * @author saharan
	 */
class Performance
{
    /**
		 * 広域衝突判定に要した時間です。
		 */
    public var broadPhaseTime : Int;
    
    /**
		 * 詳細な衝突判定に要した時間です。
		 */
    public var narrowPhaseTime : Int;
    
    /**
		 * 拘束や積分の計算に要した時間です。
		 */
    public var solvingTime : Int;
    
    /**
		 * その他の更新に要した時間です。
		 */
    public var updatingTime : Int;
    
    /**
		 * ステップ計算に要した合計時間です。
		 */
    public var totalTime : Int;
    
    /**
		 * 新しく Performance オブジェクトを作成します。
		 */
    public function new()
    {
        
    }
}

