#pragma once

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class AbstractSurfaceMapping {
    public:
        struct Setting : public common::YamlableBase {};

    protected:
        inline static std::map<std::string, std::function<std::shared_ptr<AbstractSurfaceMapping>(const std::shared_ptr<Setting> &)>> s_class_id_mapping_ = {};

    public:
        virtual ~AbstractSurfaceMapping() = default;

        // IO

        [[nodiscard]] virtual bool
        Write(const std::string &filename) const = 0;

        [[nodiscard]] virtual bool
        Write(std::ostream &s) const = 0;

        [[nodiscard]] virtual bool
        Read(const std::string &filename) = 0;

        [[nodiscard]] virtual bool
        Read(std::istream &s) = 0;

        // Factory pattern
        template<typename Derived = AbstractSurfaceMapping>
        static std::shared_ptr<Derived>
        CreateSurfaceMapping(const std::string &mapping_type, const std::shared_ptr<Setting> &setting) {
            const auto it = s_class_id_mapping_.find(mapping_type);
            if (it == s_class_id_mapping_.end()) {
                ERL_WARN("Unknown SurfaceMapping type: {}. Here are the registered SurfaceMapping types:", mapping_type);
                for (const auto &pair: s_class_id_mapping_) { ERL_WARN("  - {}", pair.first); }
                return nullptr;
            }
            auto mapping = std::dynamic_pointer_cast<Derived>(it->second(setting));
            if (mapping == nullptr) { ERL_WARN("Failed to cast SurfaceMapping type: {}", mapping_type); }
            return mapping;
        }

        template<typename Derived>
        static std::enable_if_t<std::is_base_of_v<AbstractSurfaceMapping, Derived>, bool>
        Register(std::string mapping_type = "") {
            if (mapping_type.empty()) { mapping_type = demangle(typeid(Derived).name()); }
            if (s_class_id_mapping_.find(mapping_type) != s_class_id_mapping_.end()) {
                ERL_WARN("SurfaceMapping type {} already registered", mapping_type);
                return false;
            }
            s_class_id_mapping_[mapping_type] = [](const std::shared_ptr<Setting> &setting) -> std::shared_ptr<AbstractSurfaceMapping> {
                auto mapping_setting = std::dynamic_pointer_cast<typename Derived::Setting>(setting);
                if (mapping_setting == nullptr) { mapping_setting = std::make_shared<typename Derived::Setting>(); }
                return std::make_shared<Derived>(mapping_setting);
            };
            ERL_DEBUG("{} is registered.", mapping_type);
            return true;
        }
    };

#define ERL_REGISTER_SURFACE_MAPPING(Derived) inline const volatile bool kRegistered##Derived = erl::geometry::AbstractSurfaceMapping::Register<Derived>()
}  // namespace erl::geometry
