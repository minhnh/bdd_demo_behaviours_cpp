#ifndef BDD_COLLAB_BHV__UTILS_HPP
#define BDD_COLLAB_BHV__UTILS_HPP

#include <string>
#include <map>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

inline constexpr const char LOCATED_AT_PICK_KEY[]  = "located_at_pick";
inline constexpr const char IS_HELD_KEY[]          = "is_held";
inline constexpr const char LOCATED_AT_PLACE_KEY[] = "located_at_place";

namespace bdd_collab_bhv {

using TopicConfig = std::map<std::string, std::string>;

inline TopicConfig load_topics(const std::string &file_path)
{
    try {
        YAML::Node root = YAML::LoadFile(file_path);

        TopicConfig topics;
        topics[LOCATED_AT_PICK_KEY]  = root[LOCATED_AT_PICK_KEY].as<std::string>();
        topics[IS_HELD_KEY]          = root[IS_HELD_KEY].as<std::string>();
        topics[LOCATED_AT_PLACE_KEY] = root[LOCATED_AT_PLACE_KEY].as<std::string>();

        if (
          topics[LOCATED_AT_PICK_KEY].empty() || topics[IS_HELD_KEY].empty()
          || topics[LOCATED_AT_PLACE_KEY].empty()
        ) {
            throw std::runtime_error(
              "YAML must contain non-empty values for 'located_at_pick', 'is_held', and "
              "'located_at_place'"
            );
        }

        return topics;
    } catch (const YAML::Exception &e) {
        throw std::runtime_error(std::string("Failed to parse YAML file: ") + e.what());
    }
}

} // namespace bdd_collab_bhv

#endif // BDD_COLLAB_BHV__UTILS_HPP