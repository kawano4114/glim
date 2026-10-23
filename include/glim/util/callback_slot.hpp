#pragma once

#include <vector>
#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

/**
 * @brief Callback slot to hold and trigger multiple callbacks
 */
template <typename Func>
class CallbackSlot {
public:
  CallbackSlot() {}
  ~CallbackSlot() {}

  /**
   * @brief Add a new callback
   * @param callback    Callback to be registered
   * @return int        Callback ID
   */
  int add(const std::function<Func>& callback) {
    auto entry = std::make_shared<Entry>(callback);
    std::lock_guard<std::mutex> lock(callbacks_mutex);
    callbacks.push_back(std::move(entry));
    return callbacks.size() - 1;
  }

  /**
   * @brief Disable a callback and wait for any in-flight invocation to finish
   * @param callback_id  Callback ID
   */
  void remove(int callback_id) {
    std::shared_ptr<Entry> entry;
    {
      std::lock_guard<std::mutex> lock(callbacks_mutex);
      if (callback_id < 0 || static_cast<std::size_t>(callback_id) >= callbacks.size()) {
        return;
      }
      entry = callbacks[callback_id];
    }
    entry->enabled = false;
    std::lock_guard<std::recursive_mutex> lock(entry->execution_mutex);
    entry->callback.reset();
  }

  /**
   * @brief Check if the slot has a valid callback
   * @return true   The slot has at least one valid callback
   * @return false  No valid callbacks
   */
  operator bool() const {
    for (const auto& entry : snapshot()) {
      if (entry->enabled) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Call all the registered callbacks
   * @param args  Arguments for the callbacks
   */
  template <class... Args>
  void call(Args&&... args) const {
    // Do not hold the registry mutex while calling user code: callbacks may
    // register/remove callbacks or invoke another slot recursively.
    for (const auto& entry : snapshot()) {
      if (!entry->enabled) {
        continue;
      }
      std::lock_guard<std::recursive_mutex> lock(entry->execution_mutex);
      if (entry->enabled) {
        // Keep the callable alive if it unregisters itself during invocation.
        const auto callback = entry->callback;
        (*callback)(args...);
      }
    }
  }

  /**
   * @brief Call all the registered callbacks
   * @param args  Arguments for the callbacks
   */
  template <class... Args>
  void operator()(Args&&... args) const {
    return call(args...);
  }

private:
  struct Entry {
    explicit Entry(const std::function<Func>& callback)
    : enabled(static_cast<bool>(callback)), callback(std::make_shared<std::function<Func>>(callback)) {}

    std::atomic_bool enabled;
    std::recursive_mutex execution_mutex;
    std::shared_ptr<std::function<Func>> callback;
  };

  std::vector<std::shared_ptr<Entry>> snapshot() const {
    std::lock_guard<std::mutex> lock(callbacks_mutex);
    return callbacks;
  }

  mutable std::mutex callbacks_mutex;
  std::vector<std::shared_ptr<Entry>> callbacks;
};

/**
 * @brief Own callback registrations for an object with a shorter lifetime than
 *        the callback slots. Call clear() at the beginning of its destructor,
 *        before stopping consumers or destroying state accessed by callbacks.
 * @note Register and clear connections on the owning object's lifecycle thread.
 */
class CallbackConnections {
public:
  CallbackConnections() = default;
  CallbackConnections(const CallbackConnections&) = delete;
  CallbackConnections& operator=(const CallbackConnections&) = delete;
  ~CallbackConnections() { clear(); }

  template <typename Func, typename Callback>
  int add(CallbackSlot<Func>& slot, const Callback& callback) {
    const int id = slot.add(callback);
    try {
      disconnectors.emplace_back([&slot, id] { slot.remove(id); });
    } catch (...) {
      slot.remove(id);
      throw;
    }
    return id;
  }

  void clear() {
    for (auto it = disconnectors.rbegin(); it != disconnectors.rend(); ++it) {
      (*it)();
    }
    disconnectors.clear();
  }

private:
  std::vector<std::function<void()>> disconnectors;
};
