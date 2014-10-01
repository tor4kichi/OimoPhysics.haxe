package oimo.math;

/**
 * ...
 * @author torakichi
 */
class FloatHelper
{
	public static function toFixed(val:Float, digit:Int=4) : String 
	{
		var str = "";
		if (val == 0) {
			str += "0";
		}
		
		var s = Math.abs(val - Math.floor(val)) * Math.pow(10, digit);
		for (d in 0...digit) {
			
//			str += 
		}
	
		return Std.string(val);
	}
	
	public static function toBinary(val:Float):Int
	{
		return val == 0 ? 0 : 1;
	}
}