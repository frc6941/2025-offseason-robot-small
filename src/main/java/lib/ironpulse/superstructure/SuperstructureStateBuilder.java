package lib.ironpulse.superstructure;

import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Builder for creating superstructure states by combining poses with actuator commands.
 * This class is fully public so it can be used from any robot package.
 */
public class SuperstructureStateBuilder {
    private final SuperstructurePose pose;
    private final String name;
    private final SuperstructureStateDataBuilder dataBuilder;
    
    /**
     * Public constructor.
     * @param name  the human-readable name of this state (for logging / tuning)
     * @param pose  the pose that describes joint positions, lengths, angles, etc.
     */
    public SuperstructureStateBuilder(String name, SuperstructurePose pose) {
        this.name = name;
        this.pose = pose;
        this.dataBuilder = new SuperstructureStateDataBuilder();
    }

    public SuperstructurePose getPose() {
        return pose;
    }
    
    public SuperstructureStateDataBuilder getDataBuilder() {
        return dataBuilder;
    }
    
    /**
     * Static factory to start a new builder.
     * @param name the state name
     * @param pose the initial pose
     * @return a new builder instance
     */
    public static SuperstructureStateBuilder create(String name, SuperstructurePose pose) {
        return new SuperstructureStateBuilder(name, pose);
    }
    
    /**
     * Adds a fixed numeric parameter (e.g., motor voltage, setpoint, etc.).
     * @param parameterName arbitrary key such as "shooter", "intake", "arm"
     * @param value         the value to store
     * @return this builder for chaining
     */
    public SuperstructureStateBuilder withParameter(String parameterName, double value) {
        dataBuilder.parameter(parameterName, value);
        return this;
    }
    
    /**
     * Adds a dynamic parameter via a DoubleSupplier (useful for tunable numbers).
     * @param parameterName key
     * @param supplier      supplier that yields the current value
     * @return this builder for chaining
     */
    public SuperstructureStateBuilder withParameter(String parameterName, DoubleSupplier supplier) {
        dataBuilder.parameter(parameterName, supplier);
        return this;
    }
    
    /**
     * Adds many parameters at once.
     * @param parameters map of name to DoubleSupplier
     * @return this builder for chaining
     */
    public SuperstructureStateBuilder withParameters(Map<String, DoubleSupplier> parameters) {
        dataBuilder.parameters(parameters);
        return this;
    }
    
    /**
     * Finalize the builder and produce the immutable state data.
     * @return a new SuperstructureStateData instance
     */
    public SuperstructureStateData build() {
        return dataBuilder.pose(pose).build();
    }
    
    /**
     * Convenience factory that skips the builder pattern when you just have
     * alternating name/value pairs.
     *
     * Example:
     *   SuperstructureStateData data = createState(pose,
     *       "shooter", 8.0,
     *       "indexer", () -> SmartDashboard.getNumber("IndexerSpeed", 6.0));
     *
     * @param pose        the pose
     * @param parameters  alternating (String name, DoubleSupplier|Number) pairs
     * @return            built SuperstructureStateData
     * @throws IllegalArgumentException if the pairs are malformed
     */
    public static SuperstructureStateData createState(SuperstructurePose pose, Object... parameters) {
        SuperstructureStateDataBuilder builder = new SuperstructureStateDataBuilder().pose(pose);
        
        for (int i = 0; i < parameters.length; i += 2) {
            if (i + 1 >= parameters.length) {
                throw new IllegalArgumentException("Missing value for parameter: " + parameters[i]);
            }
            String paramName = (String) parameters[i];
            Object value     = parameters[i + 1];
            
            if (value instanceof DoubleSupplier) {
                builder.parameter(paramName, (DoubleSupplier) value);
            } else if (value instanceof Number) {
                builder.parameter(paramName, ((Number) value).doubleValue());
            } else {
                throw new IllegalArgumentException(
                    "Parameter value must be DoubleSupplier or Number: " + value);
            }
        }
        
        return builder.build();
    }
}