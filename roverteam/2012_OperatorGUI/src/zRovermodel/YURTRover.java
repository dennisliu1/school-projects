package zRovermodel;

import java.util.LinkedList;

import org.w3c.dom.Document;
import org.w3c.dom.Node;

public class YURTRover extends SerializedComponent
{
	
	
	public YURTRover()
	{
		super();
	}
	
	
	public YURTRover(String name, Integer identifier, Robot._dataType dataType, 
			String data)
	{
		super(name, identifier, dataType);
		this.unsetWriteFlag();
		this.setData(data);
	}
	public YURTRover(String name, Integer identifier, Robot._dataType dataType, 
			String data, String accessType)
	{
		this(name, identifier, dataType, data);
		this.setAccessType(accessType);
	}
	public YURTRover(String name, Integer identifier, Robot._dataType dataType, 
			String data, String accessType, double lowerBound, double upperBound)
	{
		this(name, identifier, dataType, data,accessType);
		this.setLowerBound(lowerBound);
		this.setUpperBound(upperBound);
	}
	public YURTRover(String name, Integer identifier, Robot._dataType dataType,
			String data, String accessType, boolean writeFlag, 
			LinkedList<Component> components)
	{
		super(name, identifier, dataType, data, accessType, writeFlag, components);				
	}
	
	public YURTRover( YURTRover rover )
	{
		super(rover);
	}
	
	public String getSource()
	{
		String out = "";
		
		for (Component compElement : this.getComponents())
		{
			YURTRover roverElement = (YURTRover) compElement;
			if ( roverElement.getComponents().size() == 0)
			{
				if ( roverElement.getWriteFlag() == true)
				{
					out = out + "write.device#<" + this.getIdentifier() + 
						">.field#<" + roverElement.getIdentifier() + ">" +
						".set(" + roverElement.getData() + ")\n"; 
				}
				else
				{
					out = out + "read.device#<" + this.getIdentifier() + 
						">.field#<" + roverElement.getIdentifier() + ">" +
						".get()\n"; 
				}
			}
			else
			{
				out = out + roverElement.getSource();
			}
		}
		
		return out;
	}
	
	public YURTRover createComponentFromXML(Document xmlDoc)
	{
		YURTRover comp = new YURTRover();
		
		Node nameNode = xmlDoc.getFirstChild().getFirstChild();
		Node idNode = nameNode.getNextSibling();
		Node dataTypeNode = idNode.getNextSibling();
		Node dataNode = dataTypeNode.getNextSibling();
		Node accessTypeNode = dataNode.getNextSibling();
		Node componentsNode = accessTypeNode.getNextSibling();
		
		comp.setName(nameNode.getTextContent());
		comp.setIdentifier(Integer.parseInt(idNode.getTextContent()));
		comp.setDataType(dataTypeNode.getTextContent());
		comp.setData(dataNode.getTextContent());
		comp.setAccessType(accessTypeNode.getTextContent());

		Node componentNode = componentsNode.getFirstChild();
		for ( int i = 0; i < 10 && componentNode != null; i++)
		{
			comp.addComponent(comp.createComponentFromXMLNode(componentNode));
			componentNode = componentNode.getNextSibling();
		}
		//xmlDoc.getDocumentElement().get
		
		// Check for sub-Components
		
		return comp;
	}
	
	private YURTRover createComponentFromXMLNode(Node componentNode)
	{
		YURTRover comp = new YURTRover();
		// System.out.println(componentNode.getNodeName());
		Node nameNode = componentNode.getFirstChild();
		Node idNode = nameNode.getNextSibling();
		Node dataTypeNode = idNode.getNextSibling();
		Node dataNode = dataTypeNode.getNextSibling();
		Node accessTypeNode = dataNode.getNextSibling();
		Node componentsNode = accessTypeNode.getNextSibling();

		
		comp.setName(nameNode.getTextContent());
		// System.out.println(nameNode.getTextContent());
		comp.setIdentifier(Integer.parseInt(idNode.getTextContent()));
		comp.setDataType(dataTypeNode.getTextContent());
		comp.setData(dataNode.getTextContent());
		comp.setAccessType(accessTypeNode.getTextContent());
		
		if ( componentsNode == null)
		{
			return comp;
		}
		else
		{
			// System.out.println("components node = " + componentsNode.getNodeName());
		}
		Node newComponentNode = componentsNode.getFirstChild();
		// The less than 15 was added to bound the number of times this runs
		for ( int i = 0; i < 15 && newComponentNode != null; i++)
		{
			comp.addComponent(comp.createComponentFromXMLNode(newComponentNode));
			newComponentNode = newComponentNode.getNextSibling();
		}
		// System.out.println(comp.toString());
		return comp;
	}
	
	
	public static YURTRover buildRoverModel() {
		YURTRover rover = new YURTRover("Rover", 0, Component._dataType._int, "0");
			Component FrontComp = new YURTRover("Front21", 12, Component._dataType._int, "21");
			//FL
			FrontComp.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "-1", Robot.READ_ONLY));// Device ID
			FrontComp.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
			FrontComp.addComponent(new YURTRover("Brake", 12, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
			FrontComp.addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
			FrontComp.addComponent(new YURTRover("Brake", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
			FrontComp.addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			FrontComp.addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			FrontComp.addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
			//FR
			FrontComp.addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
			FrontComp.addComponent(new YURTRover("Brake", 22, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
			FrontComp.addComponent(new YURTRover("Position", 23, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
			FrontComp.addComponent(new YURTRover("Brake", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
			FrontComp.addComponent(new YURTRover("Driver Vcc", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			FrontComp.addComponent(new YURTRover("Load Current", 26, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			FrontComp.addComponent(new YURTRover("Timer", 27, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
		rover.addComponent(FrontComp);
			Component RearComp = new YURTRover("Rear22", 22, Component._dataType._int, "22");
			//RL
			RearComp.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "-1", Robot.READ_ONLY));// Device ID
			RearComp.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
			RearComp.addComponent(new YURTRover("Brake", 12, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
			RearComp.addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
			RearComp.addComponent(new YURTRover("Brake", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
			RearComp.addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			RearComp.addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			RearComp.addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
			//RR
			RearComp.addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
			RearComp.addComponent(new YURTRover("Brake", 22, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
			RearComp.addComponent(new YURTRover("Position", 23, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
			RearComp.addComponent(new YURTRover("Brake", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
			RearComp.addComponent(new YURTRover("Driver Vcc", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			RearComp.addComponent(new YURTRover("Load Current", 26, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			RearComp.addComponent(new YURTRover("Timer", 27, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
		rover.addComponent(RearComp);
		Component lunaComp = new YURTRover("Luna23", 23, Component._dataType._int, "23");
		//RL
		lunaComp.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "-1", Robot.READ_ONLY));// Device ID
		lunaComp.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
		lunaComp.addComponent(new YURTRover("Brake", 12, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
		lunaComp.addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
		lunaComp.addComponent(new YURTRover("Brake", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
		lunaComp.addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
		lunaComp.addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
		lunaComp.addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
		//RR
		lunaComp.addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
		lunaComp.addComponent(new YURTRover("Brake", 22, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
		lunaComp.addComponent(new YURTRover("Position", 23, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
		lunaComp.addComponent(new YURTRover("Brake", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
		lunaComp.addComponent(new YURTRover("Driver Vcc", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
		lunaComp.addComponent(new YURTRover("Load Current", 26, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
		lunaComp.addComponent(new YURTRover("Timer", 27, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
	rover.addComponent(lunaComp);
		Component DumpBucketExcavator = new YURTRover("Bucket110", 110, Component._dataType._int, "110");
		DumpBucketExcavator.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "-1", Robot.READ_ONLY));// Device ID
		DumpBucketExcavator.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
		DumpBucketExcavator.addComponent(new YURTRover("Break", 12, Component._dataType._int, "0", Robot.READ_WRITE, 0, 1));// [0,1]
		DumpBucketExcavator.addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
		DumpBucketExcavator.addComponent(new YURTRover("Faults", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
		DumpBucketExcavator.addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
		DumpBucketExcavator.addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
		DumpBucketExcavator.addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));
		
		DumpBucketExcavator.addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
		DumpBucketExcavator.addComponent(new YURTRover("Break", 22, Component._dataType._int, "0", Robot.READ_WRITE, 0, 1));// [0,1]
		DumpBucketExcavator.addComponent(new YURTRover("Position",23, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
		DumpBucketExcavator.addComponent(new YURTRover("Faults", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
		DumpBucketExcavator.addComponent(new YURTRover("Driver Vcc", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
		DumpBucketExcavator.addComponent(new YURTRover("Load Current", 26, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
		DumpBucketExcavator.addComponent(new YURTRover("Timer", 27, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));
	rover.addComponent(DumpBucketExcavator);
		
		return rover;
	}
//	public static YURTRover buildRoverModel() {
//		YURTRover rover = new YURTRover("Rover", 0, Component._dataType._int, "0");
//			Component FrontComp = new YURTRover("FrontComp", 21, Component._dataType._int, "21");
//			//FL
//			FrontComp.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			FrontComp.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			FrontComp.addComponent(new YURTRover("Brake", 12, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
//			FrontComp.addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
//			FrontComp.addComponent(new YURTRover("Brake", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
//			FrontComp.addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			FrontComp.addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			FrontComp.addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
//			//FR
//			FrontComp.addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			FrontComp.addComponent(new YURTRover("Brake", 22, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
//			FrontComp.addComponent(new YURTRover("Position", 23, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
//			FrontComp.addComponent(new YURTRover("Brake", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
//			FrontComp.addComponent(new YURTRover("Driver Vcc", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			FrontComp.addComponent(new YURTRover("Load Current", 26, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			FrontComp.addComponent(new YURTRover("Timer", 27, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
//		rover.addComponent(FrontComp);
//			Component RearComp = new YURTRover("RearComp", 22, Component._dataType._int, "22");
//			//RL
//			RearComp.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			RearComp.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			RearComp.addComponent(new YURTRover("Brake", 12, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
//			RearComp.addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
//			RearComp.addComponent(new YURTRover("Brake", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
//			RearComp.addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			RearComp.addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			RearComp.addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
//			//RR
//			RearComp.addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			RearComp.addComponent(new YURTRover("Brake", 22, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
//			RearComp.addComponent(new YURTRover("Position", 23, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
//			RearComp.addComponent(new YURTRover("Brake", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
//			RearComp.addComponent(new YURTRover("Driver Vcc", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			RearComp.addComponent(new YURTRover("Load Current", 26, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			RearComp.addComponent(new YURTRover("Timer", 27, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
//		rover.addComponent(RearComp);
//			Component ArmJointBase = new YURTRover("ArmJoint Base", 11, Component._dataType._int, "11");
//			ArmJointBase.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			ArmJointBase.addComponent(new YURTRover("Angle of Rotation", 11, Component._dataType._int, "32767", Robot.READ_WRITE, 0, 65535));// [0, 65535]
//		rover.addComponent(ArmJointBase);
//			Component ArmJointComp = new YURTRover("ArmJoint Shoulder", 12, Component._dataType._int, "12");
//			//Arm Joint Shoulder
//			ArmJointComp.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			ArmJointComp.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			ArmJointComp.addComponent(new YURTRover("Position", 12, Component._dataType._int, "", Robot.READ_WRITE, 0, 1023));// [0,1]
//			ArmJointComp.addComponent(new YURTRover("Brake", 13, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
//			ArmJointComp.addComponent(new YURTRover("Driver Vcc", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			ArmJointComp.addComponent(new YURTRover("Load Current", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			//Arm Joint Elbow
//			ArmJointComp.addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			ArmJointComp.addComponent(new YURTRover("Position", 22, Component._dataType._int, "", Robot.READ_WRITE, 0, 110));// [0,1]
//			ArmJointComp.addComponent(new YURTRover("Brake", 23, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
//			ArmJointComp.addComponent(new YURTRover("Driver Vcc", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			ArmJointComp.addComponent(new YURTRover("Load Current", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//		rover.addComponent(ArmJointComp);
//			Component CanfieldJoints = new YURTRover("Canfield Joints", 14, Component._dataType._int, "14");
//			CanfieldJoints.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			CanfieldJoints.addComponent(new YURTRover("Position1", 11, Component._dataType._int, "", Robot.READ_WRITE, 0, 110));//Position 1
//			CanfieldJoints.addComponent(new YURTRover("Position2", 21, Component._dataType._int, "", Robot.READ_WRITE, 0, 110));//Position 1
//			CanfieldJoints.addComponent(new YURTRover("Position3", 31, Component._dataType._int, "", Robot.READ_WRITE, 0, 110));//Position 1
//		rover.addComponent(CanfieldJoints);
//			Component Gripper = new YURTRover("Gripper", 15, Component._dataType._int, "15");
//			Gripper.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			Gripper.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			Gripper.addComponent(new YURTRover("Position", 12, Component._dataType._int, "", Robot.READ_WRITE, 0, 110));// [0,1]
//			Gripper.addComponent(new YURTRover("Faults", 13, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));// [0,1]
//			Gripper.addComponent(new YURTRover("Driver Vcc", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			Gripper.addComponent(new YURTRover("Load Current", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//		rover.addComponent(Gripper);
//			Component excavatorMotors = new YURTRover("excavator", 110, Component._dataType._int, "110");
//			excavatorMotors.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			excavatorMotors.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			excavatorMotors.addComponent(new YURTRover("Break", 12, Component._dataType._int, "0", Robot.READ_WRITE, 0, 1));// [0,1]
//			excavatorMotors.addComponent(new YURTRover("Faults", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
//			excavatorMotors.addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			excavatorMotors.addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//		rover.addComponent(excavatorMotors);
//			Component DumpBucketExcavator = new YURTRover("DumpBucketExcavator", 111, Component._dataType._int, "111");
//			DumpBucketExcavator.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			DumpBucketExcavator.addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
//			DumpBucketExcavator.addComponent(new YURTRover("Break", 12, Component._dataType._int, "0", Robot.READ_WRITE, 0, 1));// [0,1]
//			DumpBucketExcavator.addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
//			DumpBucketExcavator.addComponent(new YURTRover("Faults", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
//			DumpBucketExcavator.addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			DumpBucketExcavator.addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
//			DumpBucketExcavator.addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));
//		rover.addComponent(DumpBucketExcavator);
//			Component StatusDisplay = new YURTRover("Status Display", 101, Component._dataType._int, "101");
//			StatusDisplay.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//			StatusDisplay.addComponent(new YURTRover("Status Line", 11, Component._dataType._byte, "0", Robot.READ_WRITE, 0,20));//limit = 20 chars
//			StatusDisplay.addComponent(new YURTRover("Buzzer", 12, Component._dataType._int, "0", Robot.READ_WRITE, 0, 1));// [0,1]
//		rover.addComponent(StatusDisplay);
//		
//		//		TESTING ONLY
//			Component test2 = new YURTRover("Test2", 13, Component._dataType._int, "13");
//			test2.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//		rover.addComponent(test2);
//			Component test3 = new YURTRover("Test3", 14, Component._dataType._int, "14");
//			test3.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//		rover.addComponent(test3);
//			Component test4 = new YURTRover("Test4", 15, Component._dataType._int, "15");
//			test4.addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "0", Robot.READ_ONLY));// Device ID
//		rover.addComponent(test4);
//		return rover;
//	}

	public int getDeviceID(int did) {
		int device = -1;
		for(int i = 0; i < this.getComponents().size(); i++) {
			if(did == this.getComponents().get(i).getIdentifier()) {
				device = i;
				break;
			}
		}
		return device;
	}
	public int getFieldID(int did, int fid) {
		int field = -1;
		for(int i = 0; i < this.getComponents().get(did).getComponents().size(); i++) {
			if(fid == this.getComponents().get(did).getComponents().get(i).getIdentifier()) {
				field = i;
				break;
			}
		}
		return field;
	}
	
}//YURTRover class
