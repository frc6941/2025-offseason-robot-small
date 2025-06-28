package lib.ntext;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class NTParameterRegistry {
    private static final List<NTParameterWrapper<?>> wrappers = new ArrayList<>();
    private static final Map<NTParameterWrapper<?>, Consumer<Object>> onchangeSiConsumers = new HashMap<>();
    private static final Map<NTParameterWrapper<?>, BiConsumer<Object, Object>> onchangeBiConsumers = new HashMap<>();

    protected static void registerWrapper(NTParameterWrapper<?> wrapper) {
        wrappers.add(wrapper);
    }

    @SuppressWarnings("unchecked")
    protected static void registerOnChange(NTParameterWrapper<?> wrapper,
                                           BiConsumer<?, ?> functor) {
        BiConsumer<Object, Object> castedFunctor = (BiConsumer<Object, Object>) functor;
        onchangeBiConsumers.put(wrapper, castedFunctor);
    }

    @SuppressWarnings("unchecked")
    protected static void registerOnChange(NTParameterWrapper<?> wrapper, Consumer<?> functor) {
        Consumer<Object> castedFunctor = (Consumer<Object>) functor;
        onchangeSiConsumers.put(wrapper, castedFunctor);
    }

    protected static void unregisterOnChange(NTParameterWrapper<?> wrapper) {
        onchangeSiConsumers.remove(wrapper);
        onchangeBiConsumers.remove(wrapper);
    }

    public static void refresh() {
        for (NTParameterWrapper<?> wrapper : wrappers) {
            var siEntry = onchangeSiConsumers.get(wrapper);
            var biEntry = onchangeBiConsumers.get(wrapper);
            if (wrapper.hasChanged()) {
                if (siEntry != null) siEntry.accept(wrapper.getValue());
                if (biEntry != null) biEntry.accept(wrapper.value, wrapper.prevValue);
            }
            wrapper.refresh();
        }
    }
}
