package lib.ironpulse.superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Builder for creating SuperstructureStateData instances.
 * This builder allows fluent construction of state data with poses and parameters.
 */
public class SuperstructureStateDataBuilder {
    private SuperstructurePose pose;
    private final Map<String, DoubleSupplier> parameters = new HashMap<>();
    
    /**
     * Sets the pose for this state.
     * @param pose The superstructure pose
     * @return This builder for chaining
     */
    public SuperstructureStateDataBuilder pose(SuperstructurePose pose) {
        this.pose = pose;
        return this;
    }
    
    /**
     * Adds a parameter with a DoubleSupplier.
     * @param name The parameter name
     * @param supplier The supplier providing dynamic values
     * @return This builder for chaining
     */
    public SuperstructureStateDataBuilder parameter(String name, DoubleSupplier supplier) {
        parameters.put(name, supplier);
        return this;
    }
    
    /**
     * Adds a parameter with a fixed value.
     * @param name The parameter name
     * @param value The fixed value
     * @return This builder for chaining
     */
    public SuperstructureStateDataBuilder parameter(String name, double value) {
        parameters.put(name, () -> value);
        return this;
    }
    
    /**
     * Adds multiple parameters.
     * @param parameters Map of parameter names to suppliers
     * @return This builder for chaining
     */
    public SuperstructureStateDataBuilder parameters(Map<String, DoubleSupplier> parameters) {
        this.parameters.putAll(parameters);
        return this;
    }
    
    /**
     * Builds the final SuperstructureStateData.
     * @return A new SuperstructureStateData instance
     */
    public SuperstructureStateData build() {
        if (pose == null) {
            throw new IllegalStateException("Pose must be set before building");
        }
        return new SuperstructureStateData(pose, parameters);
    }
}