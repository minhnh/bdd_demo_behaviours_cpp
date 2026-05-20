#ifndef BDD_COLLAB_BHV__UTILS_HPP
#define BDD_COLLAB_BHV__UTILS_HPP

#include <string>
#include <map>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace bdd_collab_bhv {

using TopicConfig = std::map<std::string, std::string>;

inline TopicConfig load_topics(const std::string &file_path)
{
    try {
        YAML::Node root = YAML::LoadFile(file_path);

        TopicConfig topics;
        topics["located_at_pick"]  = root["located_at_pick"].as<std::string>();
        topics["is_held"]          = root["is_held"].as<std::string>();
        topics["located_at_place"] = root["located_at_place"].as<std::string>();

        if (
          topics["located_at_pick"].empty() || topics["is_held"].empty()
          || topics["located_at_place"].empty()
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