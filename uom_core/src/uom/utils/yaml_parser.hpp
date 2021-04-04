#pragma once

#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>


namespace YAML
{   // This has to be in the same namespace as the Emitter.
    // yaml serialization helper function for the Eigen3 Matrix object.
    // The matrix is a base class for dense matrices.
    // http://eigen.tuxfamily.org/dox-devel/TutorialMatrixClass.html
    template <class Scalar_, int A_, int B_, int C_, int D_, int E_>
    struct convert<Eigen::Matrix<Scalar_, A_, B_, C_, D_, E_> >
    {
        template <class Scalar, int A, int B, int C, int D, int E>
        static Node encode(const Eigen::Matrix<Scalar, A, B, C, D, E>& M)
        {
            Node node;
            typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
            IndexType rows = M.rows();
            IndexType cols = M.cols();
            node["rows"] = rows;
            node["cols"] = cols;
            CHECK_GT(rows, 0);
            CHECK_GT(cols, 0);
            for (IndexType i = 0; i < rows; ++i)
            {
                for (IndexType j = 0; j < cols; ++j)
                {
                    node["data"].push_back(M(i, j));
                }
            }
            return node;
        }

        template <class Scalar, int A, int B, int C, int D, int E>
        static bool decode(
            const Node& node, Eigen::Matrix<Scalar, A, B, C, D, E>& M)
        {
            typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
            IndexType rows = node["rows"].as<IndexType>();
            IndexType cols = node["cols"].as<IndexType>();

            CHECK_EQ(rows, A)
                << "Unexpected number of rows found for fixed-sized type.";
            CHECK_EQ(cols, B)
                << "Unexpected number of columns found for fixed-sized type.";

            size_t expected_size = M.rows() * M.cols();
            if (!node["data"].IsSequence() || node["data"].size() != expected_size)
            {
                LOG(ERROR) << "Data entries of matrix has wrong format.";
                return false;
            }

            YAML::const_iterator it = node["data"].begin();
            YAML::const_iterator it_end = node["data"].end();
            if (rows > 0 && cols > 0)
            {
                for (IndexType i = 0; i < rows; ++i)
                {
                    for (IndexType j = 0; j < cols; ++j)
                    {
                        CHECK(it != it_end);
                        M(i, j) = it->as<Scalar>();
                        ++it;
                    }
                }
            }
            return true;
        }

        template <class Scalar, int B, int C, int D, int E>
        static bool decode(
            const Node& node, Eigen::Matrix<Scalar, Eigen::Dynamic, B, C, D, E>& M)
        {
            typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, B, C, D, E>::Index IndexType;
            IndexType rows = node["rows"].as<IndexType>();
            IndexType cols = node["cols"].as<IndexType>();

            CHECK_EQ(cols, B)
                << "Unexpected number of columns found for fixed-sized type.";
            M.resize(rows, Eigen::NoChange);

            size_t expected_size = M.rows() * M.cols();
            if (!node["data"].IsSequence() || node["data"].size() != expected_size)
            {
                LOG(ERROR) << "Data entries of matrix has wrong format.";
                return false;
            }

            YAML::const_iterator it = node["data"].begin();
            YAML::const_iterator it_end = node["data"].end();
            if (rows > 0 && cols > 0)
            {
                for (IndexType i = 0; i < rows; ++i)
                {
                    for (IndexType j = 0; j < cols; ++j)
                    {
                        CHECK(it != it_end);
                        M(i, j) = it->as<Scalar>();
                        ++it;
                    }
                }
            }
            return true;
        }

        template <class Scalar, int A, int C, int D, int E>
        static bool decode(
            const Node& node, Eigen::Matrix<Scalar, A, Eigen::Dynamic, C, D, E>& M)
        {
            typedef typename Eigen::Matrix<Scalar, A, Eigen::Dynamic, C, D, E>::Index
                IndexType;
            IndexType rows = node["rows"].as<IndexType>();
            IndexType cols = node["cols"].as<IndexType>();

            CHECK_EQ(rows, A)
                << "Unexpected number of rows found for fixed-sized type.";

            M.resize(Eigen::NoChange, cols);

            size_t expected_size = M.rows() * M.cols();
            if (!node["data"].IsSequence() || node["data"].size() != expected_size)
            {
                LOG(ERROR) << "Data entries of matrix has wrong format.";
                return false;
            }

            YAML::const_iterator it = node["data"].begin();
            YAML::const_iterator it_end = node["data"].end();
            if (rows > 0 && cols > 0)
            {
                for (IndexType i = 0; i < rows; ++i)
                {
                    for (IndexType j = 0; j < cols; ++j)
                    {
                        CHECK(it != it_end);
                        M(i, j) = it->as<Scalar>();
                        ++it;
                    }
                }
            }
            return true;
        }

        template <class Scalar, int C, int D, int E>
        static bool decode(
            const Node& node,
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, C, D, E>& M)
        {
            typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, C, D,
                E>::Index IndexType;
            IndexType rows = node["rows"].as<IndexType>();
            IndexType cols = node["cols"].as<IndexType>();

            M.resize(rows, cols);

            size_t expected_size = M.rows() * M.cols();
            if (!node["data"].IsSequence() || node["data"].size() != expected_size)
            {
                LOG(ERROR) << "Data entries of matrix has wrong format.";
                return false;
            }

            YAML::const_iterator it = node["data"].begin();
            YAML::const_iterator it_end = node["data"].end();
            if (rows > 0 && cols > 0)
            {
                for (IndexType i = 0; i < rows; ++i)
                {
                    for (IndexType j = 0; j < cols; ++j)
                    {
                        CHECK(it != it_end);
                        M(i, j) = it->as<Scalar>();
                        ++it;
                    }
                }
            }
            return true;
        }
    };
}  // namespace YAML


class YamlParser
{
public:

    YamlParser() = delete;

    YamlParser(const std::string& yaml_path)
    {
        try
        {
            node_ = YAML::LoadFile(yaml_path);
        }
        catch (const YAML::Exception& e)
        {
            LOG(FATAL) << "Error when parsing " << yaml_path << ", Reason: " << e.what();
            std::exit(-1);
        }
    }

    template <typename T>
    void get_param(const std::string& id, T& output, bool optional = false) const
    {
        CHECK(!id.empty());


        if (YAML::Node parameter = node_[id])
        {
            output = parameter.as<T>();
        }
        else
        {
            LOG(FATAL) << "Error when read `" << id << "`, Reason: " << id << " is not found.";
        }
    }


    template <typename T>
    void get_nested_param(const std::string& id, const std::string& name, T& output, bool optional = false) const
    {
        CHECK(!id.empty());

        try
        {
            output = node_[id][name].as<T>();
        }
        catch (const YAML::Exception& e)
        {
            LOG(FATAL) << "Error when read `" << id << "`, Reason: " << e.what();
            std::exit(-1);
        }
    }


    template <typename T>
    void set_param(const std::string& id, const T& input)
    {
        try
        {
            node_[id] = input;
        }
        catch (const YAML::Exception& e)
        {
            LOG(FATAL) << "Error when write `" << id << "`, Reason: " << e.what();
            std::exit(-1);
        }
    }


private:

    YAML::Node node_;

};


