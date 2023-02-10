package commands;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;

import gui.MainWindow;
import gui_elements.Button;
import gui_elements.EventButton;
import util.Pos;

/**
 * An interactable that stores a list of other interactables of type T.
 * V is the type of the data stored in T.
 * 
 * @author Brandon C.
 */
public class InteractableList<T extends Interactable<V>, V> extends Button implements Interactable<List<T>> {

    private static final Color ADD_BUTTON_COLOR = new Color(163, 252, 139);
    private static final Color DOWN_BUTTON_COLOR = new Color(105, 177, 255);
    private static final Color REMOVE_BUTTON_COLOR = new Color(255, 112, 102);

    private static final int MARGIN = 10;
    private static final int ADD_BUTTON_HEIGHT = 40;
    private static final int REMOVE_WIDTH = 30;

    private String listName;
    private final List<T> interactables;
    private final List<RemoveButton> removeButtons;
    private final List<MoveButton> upButtons;
    private final List<MoveButton> downButtons;
    private final Supplier<T> supplier;
    private final String elementName;
    private final T dummy;
    private final boolean autopopulate;
    private final int buttonHeight;
    private final EventButton addNew;
    
    /**
     * Creates a new InteractableList.
     * @param listName The name of the interactable.
     * @param elementName The name to give each element in the list (followed by index).
     * @param autopopulate Whether or not to create new elements automatically.
     * @param supplier The supplier for new elements.
     */
    public InteractableList(String listName, String elementName, boolean autopopulate, Supplier<T> supplier) {
        super(listName, 0, 0, 0, 20);
        this.listName = listName;
        this.supplier = supplier;
        this.elementName = elementName;
        this.autopopulate = autopopulate;
        interactables = new ArrayList<T>();
        dummy = supplier.get();
        dummy.resize();
        buttonHeight = dummy.getSize().y/3;
        removeButtons = new ArrayList<RemoveButton>();
        upButtons = new ArrayList<MoveButton>();
        downButtons = new ArrayList<MoveButton>();
        if (!autopopulate) {
            addNew = new EventButton("New " + elementName, MARGIN, 20, REMOVE_WIDTH + MARGIN + dummy.getSize().x, ADD_BUTTON_HEIGHT, true) {
                @Override
                public void onClick() {
                    addInteractable(supplier.get());
                    MainWindow.getInstance().addHistoryEntry('+');
                }
            };
            addNew.setColor(ADD_BUTTON_COLOR);
            addChild(addNew);
        } else {
            addNew = null;
        }
        resize();
    }    
 
    /**
     * Adds an interactable to the end of the list.
     * @param i The interactable to add.
     */
    private void addInteractable(T i) {
        addInteractable(interactables.size(), i);
    }

    /**
     * Adds an interactable to the list. Creates the buttons to remove, move up, and move down.
     * @param index The index to put the interactable.
     * @param i The interactable to add.
     */
    private void addInteractable(int index, T i) {
        i.update(new Pos(x, y));
        interactables.add(index, i);
        RemoveButton rb = new RemoveButton(i);
        removeButtons.add(index, rb);
        MoveButton up = new MoveButton(i, true);
        upButtons.add(index, up);
        MoveButton dn = new MoveButton(i, false);
        downButtons.add(index, dn);
        addChild(i, rb, up, dn);
        resize();
        if (i instanceof PointSelect) {
            PointSelect ps = ((PointSelect) i);
            if (index < interactables.size() - 1) {
                ps.setNext((PointSelect) interactables.get(index+1));
            } else {
                ps.setNext(null);
            }
            if (index > 0) {
                ((PointSelect) interactables.get(index-1)).setNext(ps);
            }
        }
    }

    /**
     * Removes an interactable from the list. Removes the remove button, up arrow, and down arrow as well.
     * Changes the links if the elements are point selects so that they are in the proper order.
     * @param i The interactable to remove.
     */
    private void removeInteractable(T i) {
        int j = interactables.indexOf(i);
        removeChild(interactables.remove(j), removeButtons.remove(j), upButtons.remove(j), downButtons.remove(j));
        if (i instanceof PointSelect) {
            if (j > 0) {//if there is one before this
                PointSelect prev = (PointSelect) interactables.get(j-1);
                if (j < interactables.size()) {//if there is one after this
                    prev.setNext((PointSelect)interactables.get(j));
                } else {
                    prev.setNext(null);
                }
            }
        }
    }

    /**
     * Sets the relative positions of the interactables in the list.
     */
    private void resetInteractablePositions() {
        int currentY = 20 + MARGIN;
        if (!autopopulate) {
            currentY += ADD_BUTTON_HEIGHT;
        }
        for (int k = 0; k < interactables.size(); k++) {
            T i = interactables.get(k);
            i.setRelativePosition(new Pos(REMOVE_WIDTH + 2*MARGIN, currentY));
            currentY += i.getSize().y + MARGIN;
            i.setName(elementName + " " + k);
        }
    }

    /**
     * {@inheritDoc}
     * If the InteractableList is set to autopopulate it will create new elements and remove extra null elements.
     * Sets the relative positions of elements based on their position in the list.
     */
    @Override
    public void update(Pos parentPos) {
        if (autopopulate) {
            if (interactables.size() == 0 ||
                interactables.get(interactables.size()-1).getValue() != null
            ) {
                addInteractable(supplier.get());
            } 
            for (int j = interactables.size()-2; j >= 0; j--) {
                T i = interactables.get(j);
                if (i.getValue() == null) {
                    removeInteractable(i);
                }
            }
        }
        resetInteractablePositions();
        for (int j = 0; j < interactables.size(); j++) {
            T i = interactables.get(j);
            i.resize();
            if (removeButtons.size() > j) {
                Button rb = removeButtons.get(j);
                rb.setRelativePosition(new Pos(i.getRelativePosition().x - MARGIN - REMOVE_WIDTH, i.getRelativePosition().y + buttonHeight));
                Button up = upButtons.get(j);
                up.setRelativePosition(new Pos(i.getRelativePosition().x - MARGIN - REMOVE_WIDTH, i.getRelativePosition().y));
                Button dn = downButtons.get(j);
                dn.setRelativePosition(new Pos(i.getRelativePosition().x - MARGIN - REMOVE_WIDTH, i.getRelativePosition().y + 2*buttonHeight));
            }
        }
        super.update(parentPos);
    }

    @Override
    public List<T> getValue() {
        return interactables;
    }

    @Override 
    public void setValue(List<T> value) {
        for (int i = interactables.size()-1; i >= 0; i--) {
            removeInteractable(interactables.get(i));
        }
        for (int i = 0; i < value.size(); i++) {
            addInteractable(value.get(i));
        }
    }

    @Override
    public void setFromJson(JSONObject json) {
        JSONArray array = json.getJSONArray(getName());
        for (int j = 0; j < array.length(); j++) {
            T i = supplier.get();
            i.setFromJson(array.getJSONObject(j));
            addInteractable(i);
        }
    }

    @Override
    public JSONObject addToJson(JSONObject json) {
        JSONArray array = new JSONArray();
        for (int i = 0; i < interactables.size(); i++) {
            JSONObject o = new JSONObject();
            o = interactables.get(i).addToJson(o);
            if (o != null) {
                array.put(o);
            }
        }
        json.put(getName(), array);
        return json;
    }

	@Override
	public String getName() {
		return listName;
    }
    
	@Override
	public void setName(String name) {
		listName = name;
	}

    /**
     * A button for removing elements of the list.
     */
    private class RemoveButton extends EventButton{
        private T interactable;

        /**
         * Create a new remove button.
         * @param interactable The interactable to remove.
         */
        public RemoveButton(T interactable) {
            super("X", 0, 0, REMOVE_WIDTH, buttonHeight, true);
            this.interactable = interactable;
            setColor(REMOVE_BUTTON_COLOR);
        }

        /**
         * Removes the interactable from the list.
         */
        @Override
        public void onClick() {
            InteractableList.this.removeInteractable(interactable);
            MainWindow.getInstance().addHistoryEntry('-');
        }
    }

    /**
     * A button for moving elements of a list.
     */
    private class MoveButton extends EventButton{
        private T interactable;
        private boolean isUp;

         /**
         * Create a new MoveButton.
         * @param interactable The interactable the button is linked to.
         * @param isUp true = up, false = down;
         */
        public MoveButton(T interactable, boolean isUp) {
            super("^", 0, 0, REMOVE_WIDTH, buttonHeight, true);
            this.interactable = interactable;
            this.isUp = isUp;
            if (isUp) {
                setColor(ADD_BUTTON_COLOR);
            } else {
                setColor(DOWN_BUTTON_COLOR);
                setText("v");
            }
            
        }

        /**
         * Moves the interactable up or down depending on whether it is an up button or not.
         */
        @Override
        public void onClick() {
            int index = interactables.indexOf(interactable);
            if (isUp) {
                if (index != 0) {
                    InteractableList.this.removeInteractable(interactable);
                    InteractableList.this.addInteractable(index - 1, interactable);
                }
            } else {
                if (index != interactables.size()-1) {
                    InteractableList.this.removeInteractable(interactable);
                    InteractableList.this.addInteractable(index + 1, interactable);
                }
            }
        }
    }
}