package protocols;

public class DefualtProtocol {
	private String operateType;
	private Param param;
	private Param param2;
	private int timestamp;
	private double[] jointPos = new double[7];
	
	public String getOperateType() {
		return operateType;
	}
	public void setOperateType(String operateType) {
		this.operateType = operateType;
	}
	public Param getParam() {
		return param;
	}
	public void setParam(Param param) {
		this.param = param;
	}
	public Param getParam2() {
		return param2;
	}
	public void setParam2(Param param) {
		this.param2 = param;
	}
	public int getTimestamp() {
		return timestamp;
	}
	public void setTimestamp(int timestamp) {
		this.timestamp = timestamp;
	}
	
	public double[] getJointPos() {
		return jointPos;
	}
	
	/*@Override
	public String toString() {
		return "DefualtProtocol [OperateType=" + OperateType + ", param=" + param.toString() + ", timestamp=" + timestamp + "]";
	}*/
}
