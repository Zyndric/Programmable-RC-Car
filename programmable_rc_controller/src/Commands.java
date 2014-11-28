public enum Commands {
	INVALID  (0, 0),
	DRIVE   (10, 0),
	RECORD  (18, 0),
	REPLAY  (96, 1),
	STOP   (129, 2);
	
	public final int cmdCode;
	public final int imgIndex;
	
	Commands(final int cmdCode, final int imgIndex)
	{
		this.cmdCode = cmdCode;
		this.imgIndex = imgIndex;
	}
}