package vcsc.core.component;

import java.util.ArrayList;
import java.util.List;

public interface ComponentGroup<T extends Component> extends Component {

    /**
     * Get the component at the specified index.
     * @param index The index of the component.
     * @return Component at the specified index.
     */
    T getComponentByIndex(int index);

    /**
     * Add a component to this group.
     * @param component The component to add.
     */
    void addComponent(T component);

    /**
     * Add multiple components to this group.
     * @param components The components to add.
     */
    void addComponents(T... components);

    /**
     * Get the list of components in this group.
     * @return List<Component>
     */
    List<T> getComponents();

    /**
     * Update this component group for the current iteration.
     */
    void update();
}
