package commands;

import org.json.JSONObject;

import gui_elements.Element;

/**
 * An interface that provides functionality for loading from and exporting information to JSON format.
 * T is the type of the value stored in the Interactable.
 * 
 * @author Brandon C.
 */
public interface Interactable<T> extends Element{

    /**
     * Sets the value of the interactable from JSON.
     * @param json The JSONObject storing the information for the parameters of the command.
     */
    public void setFromJson(JSONObject json);

    /**
     * Adds this interactable's value to the JSONObject with the parameters of the command.
     * @param json The JSONObject to add to.
     * @return The edited JSONObject.
     */
    public JSONObject addToJson(JSONObject json);

    /**
     * Gets the name of the interactable, used as the key when added to a JSONObject.
     * @return the name.
     */
    public String getName();

    /**
     * Sets the name of the interactable.
     * @param name The name to use.
     */
    public void setName(String name);

    /**
     * Gets the value of the interactable.
     * @return the Value of type T stored in the interactable.
     */
    public T getValue();
    
    /**
     * Sets the value of the interactable.
     * @param the Value of type T to be used.
     */
    public void setValue(T value);
}