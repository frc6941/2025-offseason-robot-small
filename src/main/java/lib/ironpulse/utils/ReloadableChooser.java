// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.ironpulse.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

/**
 * The {@link ReloadableChooser} class is a useful tool for presenting a selection of options to the
 * {@link SmartDashboard}. Options can be added, removed, or cleared dynamically.
 *
 * @param <V> The type of the values to be stored
 */
public class ReloadableChooser<V> implements Sendable, AutoCloseable {
    private static final String DEFAULT = "default";
    private static final String SELECTED = "selected";
    private static final String ACTIVE = "active";
    private static final String OPTIONS = "options";
    private static final String INSTANCE = ".instance";

    private final Map<String, V> m_map = new LinkedHashMap<>();
    private String m_defaultChoice = "";
    private final int m_instance;
    private String m_previousVal;
    private Consumer<V> m_listener;
    private String m_selected;
    private final ReentrantLock m_mutex = new ReentrantLock();
    private static final AtomicInteger s_instances = new AtomicInteger();

    @SuppressWarnings("this-escape")
    public ReloadableChooser() {
        m_instance = s_instances.getAndIncrement();
        SendableRegistry.add(this, "SendableChooser", m_instance);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Adds the given object to the list of options.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void addOption(String name, V object) {
        m_map.put(name, object);
    }

    /**
     * Adds the given object and marks it as the default.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void setDefaultOption(String name, V object) {
        requireNonNullParam(name, "name", "setDefaultOption");
        m_defaultChoice = name;
        addOption(name, object);
    }

    /**
     * Removes the given option. If it was the default or currently selected option,
     * those are reset.
     *
     * @param name the name of the option to remove
     */
    public void removeOption(String name) {
        requireNonNullParam(name, "name", "removeOption");
        m_map.remove(name);
        if (name.equals(m_defaultChoice)) {
            m_defaultChoice = "";
        }
        m_mutex.lock();
        try {
            if (name.equals(m_selected)) {
                m_selected = null;
            }
            m_previousVal = null;
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Clears all options and resets default and selected choices.
     */
    public void clear() {
        m_map.clear();
        m_defaultChoice = "";
        m_mutex.lock();
        try {
            m_selected = null;
            m_previousVal = null;
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Returns the selected option, or the default if none selected.
     */
    public V getSelected() {
        m_mutex.lock();
        try {
            return (m_selected != null) ? m_map.get(m_selected) : m_map.get(m_defaultChoice);
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Binds a listener that's called when the selected value changes.
     *
     * @param listener function accepting the new value
     */
    public void onChange(Consumer<V> listener) {
        requireNonNullParam(listener, "listener", "onChange");
        m_mutex.lock();
        m_listener = listener;
        m_mutex.unlock();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.publishConstInteger(INSTANCE, m_instance);
        builder.addStringProperty(DEFAULT, () -> m_defaultChoice, null);
        builder.addStringArrayProperty(OPTIONS, () -> m_map.keySet().toArray(new String[0]), null);
        builder.addStringProperty(
                ACTIVE,
                () -> {
                    m_mutex.lock();
                    try {
                        return (m_selected != null) ? m_selected : m_defaultChoice;
                    } finally {
                        m_mutex.unlock();
                    }
                },
                null);
        builder.addStringProperty(
                SELECTED,
                null,
                val -> {
                    V choice;
                    Consumer<V> listener;
                    m_mutex.lock();
                    try {
                        m_selected = val;
                        if (!val.equals(m_previousVal) && m_listener != null) {
                            choice = m_map.get(val);
                            listener = m_listener;
                        } else {
                            choice = null;
                            listener = null;
                        }
                        m_previousVal = val;
                    } finally {
                        m_mutex.unlock();
                    }
                    if (listener != null) {
                        listener.accept(choice);
                    }
                });
    }
}
