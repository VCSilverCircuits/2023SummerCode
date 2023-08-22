package vcsc.core.assembly;

import java.util.ArrayList;
import java.util.List;

import vcsc.core.component.Component;
import vcsc.core.component.ComponentGroup;

public class Assembly implements ComponentGroup<Component> {
    List<Component> components = new ArrayList<>();

    /**
     * Initialize this component group.
     */
    @Override
    public void init() {
        for (Component component : components) {
            component.init();
        }
    }

    /**
     * Get the component at the specified index.
     * @param index The index of the component.
     * @return Component at the specified index.
     */
    public Component getComponentByIndex(int index) {
        return components.get(index);
    }

    /**
     * Add a component to this group.
     * @param component The component to add.
     */
    public void addComponent(Component component) {
        components.add(component);
    }

    /**
     * Add multiple components to this group.
     * @param components The components to add.
     */
    public void addComponents(Component... components) {
        for (Component component : components) {
            addComponent(component);
        }
    }

    /**
     * Get the list of components in this group.
     * @return List<Component>
     */
    public List<Component> getComponents() {
        return components;
    }

    /**
     * Update this component group for the current iteration.
     */
    public void update() {
        for (Component component : components) {
            component.update();
        }
    }
}