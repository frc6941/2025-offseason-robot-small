package lib.ironpulse.superstructure;

import java.util.Collections;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Represents a named pose configuration for a superstructure mechanism.
 * Contains a map of named parameters that can represent joint positions, angles, or any other pose-related values.
 */
public class SuperstructurePose{
    private final String name;
    private final Map<String, DoubleSupplier> parameters;
    
    /**
     * Creates a new SuperstructurePose.
     * @param name The name of this pose
     * @param parameters Map of parameter names to their DoubleSuppliers
     */
    public SuperstructurePose(String name, Map<String, DoubleSupplier> parameters) {
        this.name = name;
        this.parameters = Collections.unmodifiableMap(parameters);
    }
    
    /**
     * Gets the name of this pose.
     * @return The pose name
     */
    public String getName() {
        return name;
    }
    
    /**
     * Gets a parameter value by name.
     * @param parameterName The name of the parameter
     * @return The current value of the parameter
     */
    public double getParameter(String parameterName) {
        return parameters.get(parameterName).getAsDouble();
    }
    
    /**
     * Gets a parameter supplier by name.
     * @param parameterName The name of the parameter
     * @return The DoubleSupplier for this parameter
     */
    public DoubleSupplier getParameterSupplier(String parameterName) {
        return parameters.get(parameterName);
    }
    
    /**
     * Checks if this pose contains a parameter.
     * @param parameterName The name to check
     * @return true if the parameter exists
     */
    public boolean hasParameter(String parameterName) {
        return parameters.containsKey(parameterName);
    }
    
    /**
     * Gets all parameter names for this pose.
     * @return Set of parameter names
     */
    public java.util.Set<String> getParameterNames() {
        return parameters.keySet();
    }

    public static SuperstructurePoseBuilder builder(String name) {
        return SuperstructurePoseBuilder.create(name);
    }
}