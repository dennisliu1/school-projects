package zRovermodel;

import java.io.File;
import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import aiInterface.testing.client.TestingUtil;

public class YURTRoverExample
{

	public static void main(String[] args)
	{
		Component rover;
		String name = "E.V.E.";
		Integer id = 0;
		Component._dataType dataType = Component._dataType._int;
		String data = "45";
		
		rover = new YURTRover(name, id, dataType, data);
		
		System.out.println(rover.toString());

		String armName = "roverArm";
		Integer armID = 1;
		Robot._dataType armDataType = Robot._dataType._double;
		String armData = "345";
		
		Component roverArm = new YURTRover(armName, armID, armDataType, armData);
		
		System.out.println(roverArm.toString());
		rover.addComponent(roverArm);
		System.out.println(rover.toString());
		
		/*
		 * Lets create a Robot from a XML file now
		 */
		Document doc = null;
		String roverFileName = "C:\\NBWorkspace\\OperatorGUI\\src\\zRovermodel\\YURTRoverExample.xml";
		try
		{
			doc = TestingUtil.parseXmlFile(roverFileName);
			
		} catch (IOException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ParserConfigurationException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (SAXException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.out.println(doc.getFirstChild().getFirstChild().getNodeName());
		System.out.println(doc.getDocumentElement().getNodeName());
		Node roverNodeName = doc.getDocumentElement().getElementsByTagName("name").item(0);
		Node roverNodeID = doc.getDocumentElement().getElementsByTagName("ID").item(0);
		Node roverNodedataType = doc.getDocumentElement().getElementsByTagName("dataType").item(0);
		Node roverNodeComponents = doc.getDocumentElement().getElementsByTagName("Components").item(0);
		System.out.println(doc.getDocumentElement().getElementsByTagName("name").item(0).getNodeName());
		System.out.println(doc.getDocumentElement().getElementsByTagName("name").item(0).getTextContent());
		
		System.out.println(roverNodeComponents.getFirstChild().getFirstChild().getTextContent());
		
		Node nameNode = doc.getFirstChild().getFirstChild();
		Node idNode = nameNode.getNextSibling();
		System.out.println(idNode.getNodeName());
		System.out.println(idNode.getTextContent());
		
		Component rover2 = rover.createComponentFromXML(doc);
		
		System.out.println(rover2.toString());
		System.out.println(rover2.getSource());
		
	}
}
