package zRovermodel;

import java.util.LinkedList;

import org.w3c.dom.Document;
import org.w3c.dom.Node;

import aiInterface.data.Data;
import aiInterface.data.DataContainer;
import aiInterface.data.conversion.Converter;

public class Component extends Robot
{

	private LinkedList<Component> components;
	
	public Component()
	{
		super();
		this.components = new LinkedList<Component>();
	}
	/**
	 * A Component is a part of the cover that can have sub
	 * @param name
	 * @param identifier
	 * @param dataType
	 */
	public Component(String name, Integer identifier, Robot._dataType dataType, String accessType)
	{
		this();
		this.setName(name);
		this.setIdentifier(identifier);
		this.setDataType(dataType);
		this.setAccessType(accessType);
	}
	public Component(String name, Integer identifier, Robot._dataType dataType)
	{
		this(name, identifier, dataType, Robot.READ_ONLY);
	}
	
	/**
	 * This will construct a Component using the passed items. A shallow copy
	 * of the components will be used.
	 * @param name
	 * @param identifier
	 * @param dataType
	 * @param components
	 */
	public Component(String name, Integer identifier, Robot._dataType dataType, 
			String data, String accessType,  LinkedList<Component> components)
	{
		this(name, identifier, dataType);
		this.setData(data);
		this.setAccessType(accessType);
		this.setComponents(components);
	}
	
	/**
	 * This will make a shallow copy of the components passed
	 * @param component
	 */
	public Component(Component component)
	{
		this(component.getName(), component.getIdentifier(), 
				component.getDataType(), component.getData(),
				component.getAccessType(), component.getComponents());
	}
	
	public void addComponent(Component component)
	{
		// It is important to not make a deep copy of this object.
		components.add(component);
	}
	
	/**
	 * This returns a shallow copy but that should be more than enough
	 * for what we are working with.
	 * @return
	 */
	public LinkedList<Component> getComponents()
	{
		return this.components;
	}
	
	protected void setComponents(LinkedList<Component> components)
	{
		/*
		LinkedList<? extends Robot> comps = new LinkedList<Component>();
		for ( Component comp : components)
		{
			comps.add(comp);
		}
		*/
		this.components = components;
	}
	
	public Component createComponentFromXML(Document xmlDoc)
	{
		// CreateComponent
		
		Component comp = new Component();
		
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
	
	private Component createComponentFromXMLNode(Node componentNode)
	{
		Component comp = new Component();
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
	

	/*
	 * Leave these methods for now I will  fill them in later.
	 */
	
	
	@Override
	public DataContainer convertToDataContainer(String source)
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Data Convert(DataContainer convert, Converter converter, Data data)
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public DataContainer convert(DataContainer dataContainer)
	{
		// TODO Auto-generated method stub
		return null;
	}

	public String toString()
	{
		String out = "";
		out = out + super.toString();
		out = out + "components = " + this.getComponents().toString() + "\n";
		return out;
		
	}
}

