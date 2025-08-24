package lib.ironpulse.superstructure;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Immutable data container for superstructure states including poses and actuator commands.
 * Contains a pose and arbitrary named parameters for actuator control.
 */
public class SuperstructureStateData {
    private final SuperstructurePose pose;
    private final Map<String, DoubleSupplier> parameters;
    
    /**
     * Creates a new SuperstructureStateData.
     * @param pose The superstructure pose
     * @param parameters Map of parameter names to their DoubleSuppliers
     */
    public SuperstructureStateData(SuperstructurePose pose, Map<String, DoubleSupplier> parameters) {
        this.pose = pose;
        this.parameters = Collections.unmodifiableMap(new HashMap<>(parameters));
    }
    
    /**
     * Gets the pose for this state.
     * @return The superstructure pose
     */
    public SuperstructurePose getPose() {
        return pose;
    }
    
    /**
     * Gets a parameter supplier by name.
     * @param parameterName The name of the parameter
     * @return The DoubleSupplier for this parameter, or () -> 0.0 if not found
     */
    public DoubleSupplier getParameter(String parameterName) {
        return parameters.getOrDefault(parameterName, () -> 0.0);
    }
    
    /**
     * Gets the value of a parameter.
     * @param parameterName The name of the parameter
     * @return The current value of the parameter
     */
    public double getParameterValue(String parameterName) {
        return getParameter(parameterName).getAsDouble();
    }
    
    /**
     * Gets all parameters for this state.
     * @return Map of parameter names to suppliers
     */
    public Map<String, DoubleSupplier> getParameters() {
        return parameters;
    }
    
    /**
     * Creates a new builder for SuperstructureStateData.
     * @return A new builder instance
     */
    public static SuperstructureStateDataBuilder builder() {
        return new SuperstructureStateDataBuilder();
    }
}