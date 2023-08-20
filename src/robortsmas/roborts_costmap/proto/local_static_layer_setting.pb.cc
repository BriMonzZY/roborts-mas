// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: local_static_layer_setting.proto

#include "local_static_layer_setting.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace roborts_costmap {
class ParaLocalStaticLayerDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ParaLocalStaticLayer>
      _instance;
} _ParaLocalStaticLayer_default_instance_;
}  // namespace roborts_costmap
namespace protobuf_local_5fstatic_5flayer_5fsetting_2eproto {
static void InitDefaultsParaLocalStaticLayer() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::roborts_costmap::_ParaLocalStaticLayer_default_instance_;
    new (ptr) ::roborts_costmap::ParaLocalStaticLayer();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::roborts_costmap::ParaLocalStaticLayer::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_ParaLocalStaticLayer =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsParaLocalStaticLayer}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_ParaLocalStaticLayer.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, first_map_only_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, subscribe_to_updates_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, track_unknown_space_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, use_maximum_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, unknown_cost_value_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, trinary_map_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, lethal_threshold_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, topic_name_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, is_raw_rosmessage_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::roborts_costmap::ParaLocalStaticLayer, is_debug_),
  1,
  2,
  3,
  4,
  5,
  7,
  6,
  0,
  8,
  9,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 15, sizeof(::roborts_costmap::ParaLocalStaticLayer)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::roborts_costmap::_ParaLocalStaticLayer_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "local_static_layer_setting.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n local_static_layer_setting.proto\022\017robo"
      "rts_costmap\"\212\002\n\024ParaLocalStaticLayer\022\026\n\016"
      "first_map_only\030\001 \002(\010\022\034\n\024subscribe_to_upd"
      "ates\030\002 \002(\010\022\033\n\023track_unknown_space\030\003 \002(\010\022"
      "\023\n\013use_maximum\030\004 \002(\010\022\032\n\022unknown_cost_val"
      "ue\030\005 \002(\005\022\023\n\013trinary_map\030\006 \002(\010\022\030\n\020lethal_"
      "threshold\030\007 \002(\005\022\022\n\ntopic_name\030\010 \002(\t\022\031\n\021i"
      "s_raw_rosmessage\030\t \002(\010\022\020\n\010is_debug\030\n \002(\010"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 320);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "local_static_layer_setting.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_local_5fstatic_5flayer_5fsetting_2eproto
namespace roborts_costmap {

// ===================================================================

void ParaLocalStaticLayer::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ParaLocalStaticLayer::kFirstMapOnlyFieldNumber;
const int ParaLocalStaticLayer::kSubscribeToUpdatesFieldNumber;
const int ParaLocalStaticLayer::kTrackUnknownSpaceFieldNumber;
const int ParaLocalStaticLayer::kUseMaximumFieldNumber;
const int ParaLocalStaticLayer::kUnknownCostValueFieldNumber;
const int ParaLocalStaticLayer::kTrinaryMapFieldNumber;
const int ParaLocalStaticLayer::kLethalThresholdFieldNumber;
const int ParaLocalStaticLayer::kTopicNameFieldNumber;
const int ParaLocalStaticLayer::kIsRawRosmessageFieldNumber;
const int ParaLocalStaticLayer::kIsDebugFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ParaLocalStaticLayer::ParaLocalStaticLayer()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_local_5fstatic_5flayer_5fsetting_2eproto::scc_info_ParaLocalStaticLayer.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:roborts_costmap.ParaLocalStaticLayer)
}
ParaLocalStaticLayer::ParaLocalStaticLayer(const ParaLocalStaticLayer& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  topic_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_topic_name()) {
    topic_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.topic_name_);
  }
  ::memcpy(&first_map_only_, &from.first_map_only_,
    static_cast<size_t>(reinterpret_cast<char*>(&is_debug_) -
    reinterpret_cast<char*>(&first_map_only_)) + sizeof(is_debug_));
  // @@protoc_insertion_point(copy_constructor:roborts_costmap.ParaLocalStaticLayer)
}

void ParaLocalStaticLayer::SharedCtor() {
  topic_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&first_map_only_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&is_debug_) -
      reinterpret_cast<char*>(&first_map_only_)) + sizeof(is_debug_));
}

ParaLocalStaticLayer::~ParaLocalStaticLayer() {
  // @@protoc_insertion_point(destructor:roborts_costmap.ParaLocalStaticLayer)
  SharedDtor();
}

void ParaLocalStaticLayer::SharedDtor() {
  topic_name_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void ParaLocalStaticLayer::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ParaLocalStaticLayer::descriptor() {
  ::protobuf_local_5fstatic_5flayer_5fsetting_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_local_5fstatic_5flayer_5fsetting_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ParaLocalStaticLayer& ParaLocalStaticLayer::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_local_5fstatic_5flayer_5fsetting_2eproto::scc_info_ParaLocalStaticLayer.base);
  return *internal_default_instance();
}


void ParaLocalStaticLayer::Clear() {
// @@protoc_insertion_point(message_clear_start:roborts_costmap.ParaLocalStaticLayer)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    topic_name_.ClearNonDefaultToEmptyNoArena();
  }
  if (cached_has_bits & 254u) {
    ::memset(&first_map_only_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&trinary_map_) -
        reinterpret_cast<char*>(&first_map_only_)) + sizeof(trinary_map_));
  }
  if (cached_has_bits & 768u) {
    ::memset(&is_raw_rosmessage_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&is_debug_) -
        reinterpret_cast<char*>(&is_raw_rosmessage_)) + sizeof(is_debug_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool ParaLocalStaticLayer::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:roborts_costmap.ParaLocalStaticLayer)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required bool first_map_only = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          set_has_first_map_only();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &first_map_only_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required bool subscribe_to_updates = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          set_has_subscribe_to_updates();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &subscribe_to_updates_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required bool track_unknown_space = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          set_has_track_unknown_space();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &track_unknown_space_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required bool use_maximum = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(32u /* 32 & 0xFF */)) {
          set_has_use_maximum();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &use_maximum_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required int32 unknown_cost_value = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {
          set_has_unknown_cost_value();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &unknown_cost_value_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required bool trinary_map = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(48u /* 48 & 0xFF */)) {
          set_has_trinary_map();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &trinary_map_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required int32 lethal_threshold = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(56u /* 56 & 0xFF */)) {
          set_has_lethal_threshold();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &lethal_threshold_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required string topic_name = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(66u /* 66 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_topic_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->topic_name().data(), static_cast<int>(this->topic_name().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "roborts_costmap.ParaLocalStaticLayer.topic_name");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required bool is_raw_rosmessage = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(72u /* 72 & 0xFF */)) {
          set_has_is_raw_rosmessage();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &is_raw_rosmessage_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required bool is_debug = 10;
      case 10: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(80u /* 80 & 0xFF */)) {
          set_has_is_debug();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &is_debug_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:roborts_costmap.ParaLocalStaticLayer)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:roborts_costmap.ParaLocalStaticLayer)
  return false;
#undef DO_
}

void ParaLocalStaticLayer::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:roborts_costmap.ParaLocalStaticLayer)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required bool first_map_only = 1;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(1, this->first_map_only(), output);
  }

  // required bool subscribe_to_updates = 2;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(2, this->subscribe_to_updates(), output);
  }

  // required bool track_unknown_space = 3;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(3, this->track_unknown_space(), output);
  }

  // required bool use_maximum = 4;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(4, this->use_maximum(), output);
  }

  // required int32 unknown_cost_value = 5;
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(5, this->unknown_cost_value(), output);
  }

  // required bool trinary_map = 6;
  if (cached_has_bits & 0x00000080u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(6, this->trinary_map(), output);
  }

  // required int32 lethal_threshold = 7;
  if (cached_has_bits & 0x00000040u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(7, this->lethal_threshold(), output);
  }

  // required string topic_name = 8;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->topic_name().data(), static_cast<int>(this->topic_name().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "roborts_costmap.ParaLocalStaticLayer.topic_name");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      8, this->topic_name(), output);
  }

  // required bool is_raw_rosmessage = 9;
  if (cached_has_bits & 0x00000100u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(9, this->is_raw_rosmessage(), output);
  }

  // required bool is_debug = 10;
  if (cached_has_bits & 0x00000200u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(10, this->is_debug(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:roborts_costmap.ParaLocalStaticLayer)
}

::google::protobuf::uint8* ParaLocalStaticLayer::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:roborts_costmap.ParaLocalStaticLayer)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required bool first_map_only = 1;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(1, this->first_map_only(), target);
  }

  // required bool subscribe_to_updates = 2;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(2, this->subscribe_to_updates(), target);
  }

  // required bool track_unknown_space = 3;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(3, this->track_unknown_space(), target);
  }

  // required bool use_maximum = 4;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(4, this->use_maximum(), target);
  }

  // required int32 unknown_cost_value = 5;
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(5, this->unknown_cost_value(), target);
  }

  // required bool trinary_map = 6;
  if (cached_has_bits & 0x00000080u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(6, this->trinary_map(), target);
  }

  // required int32 lethal_threshold = 7;
  if (cached_has_bits & 0x00000040u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(7, this->lethal_threshold(), target);
  }

  // required string topic_name = 8;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->topic_name().data(), static_cast<int>(this->topic_name().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "roborts_costmap.ParaLocalStaticLayer.topic_name");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        8, this->topic_name(), target);
  }

  // required bool is_raw_rosmessage = 9;
  if (cached_has_bits & 0x00000100u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(9, this->is_raw_rosmessage(), target);
  }

  // required bool is_debug = 10;
  if (cached_has_bits & 0x00000200u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(10, this->is_debug(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:roborts_costmap.ParaLocalStaticLayer)
  return target;
}

size_t ParaLocalStaticLayer::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:roborts_costmap.ParaLocalStaticLayer)
  size_t total_size = 0;

  if (has_topic_name()) {
    // required string topic_name = 8;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->topic_name());
  }

  if (has_first_map_only()) {
    // required bool first_map_only = 1;
    total_size += 1 + 1;
  }

  if (has_subscribe_to_updates()) {
    // required bool subscribe_to_updates = 2;
    total_size += 1 + 1;
  }

  if (has_track_unknown_space()) {
    // required bool track_unknown_space = 3;
    total_size += 1 + 1;
  }

  if (has_use_maximum()) {
    // required bool use_maximum = 4;
    total_size += 1 + 1;
  }

  if (has_unknown_cost_value()) {
    // required int32 unknown_cost_value = 5;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->unknown_cost_value());
  }

  if (has_lethal_threshold()) {
    // required int32 lethal_threshold = 7;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->lethal_threshold());
  }

  if (has_trinary_map()) {
    // required bool trinary_map = 6;
    total_size += 1 + 1;
  }

  if (has_is_raw_rosmessage()) {
    // required bool is_raw_rosmessage = 9;
    total_size += 1 + 1;
  }

  if (has_is_debug()) {
    // required bool is_debug = 10;
    total_size += 1 + 1;
  }

  return total_size;
}
size_t ParaLocalStaticLayer::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:roborts_costmap.ParaLocalStaticLayer)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x000003ff) ^ 0x000003ff) == 0) {  // All required fields are present.
    // required string topic_name = 8;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->topic_name());

    // required bool first_map_only = 1;
    total_size += 1 + 1;

    // required bool subscribe_to_updates = 2;
    total_size += 1 + 1;

    // required bool track_unknown_space = 3;
    total_size += 1 + 1;

    // required bool use_maximum = 4;
    total_size += 1 + 1;

    // required int32 unknown_cost_value = 5;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->unknown_cost_value());

    // required int32 lethal_threshold = 7;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->lethal_threshold());

    // required bool trinary_map = 6;
    total_size += 1 + 1;

    // required bool is_raw_rosmessage = 9;
    total_size += 1 + 1;

    // required bool is_debug = 10;
    total_size += 1 + 1;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ParaLocalStaticLayer::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:roborts_costmap.ParaLocalStaticLayer)
  GOOGLE_DCHECK_NE(&from, this);
  const ParaLocalStaticLayer* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ParaLocalStaticLayer>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:roborts_costmap.ParaLocalStaticLayer)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:roborts_costmap.ParaLocalStaticLayer)
    MergeFrom(*source);
  }
}

void ParaLocalStaticLayer::MergeFrom(const ParaLocalStaticLayer& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:roborts_costmap.ParaLocalStaticLayer)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 255u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_topic_name();
      topic_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.topic_name_);
    }
    if (cached_has_bits & 0x00000002u) {
      first_map_only_ = from.first_map_only_;
    }
    if (cached_has_bits & 0x00000004u) {
      subscribe_to_updates_ = from.subscribe_to_updates_;
    }
    if (cached_has_bits & 0x00000008u) {
      track_unknown_space_ = from.track_unknown_space_;
    }
    if (cached_has_bits & 0x00000010u) {
      use_maximum_ = from.use_maximum_;
    }
    if (cached_has_bits & 0x00000020u) {
      unknown_cost_value_ = from.unknown_cost_value_;
    }
    if (cached_has_bits & 0x00000040u) {
      lethal_threshold_ = from.lethal_threshold_;
    }
    if (cached_has_bits & 0x00000080u) {
      trinary_map_ = from.trinary_map_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 768u) {
    if (cached_has_bits & 0x00000100u) {
      is_raw_rosmessage_ = from.is_raw_rosmessage_;
    }
    if (cached_has_bits & 0x00000200u) {
      is_debug_ = from.is_debug_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ParaLocalStaticLayer::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:roborts_costmap.ParaLocalStaticLayer)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ParaLocalStaticLayer::CopyFrom(const ParaLocalStaticLayer& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:roborts_costmap.ParaLocalStaticLayer)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ParaLocalStaticLayer::IsInitialized() const {
  if ((_has_bits_[0] & 0x000003ff) != 0x000003ff) return false;
  return true;
}

void ParaLocalStaticLayer::Swap(ParaLocalStaticLayer* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ParaLocalStaticLayer::InternalSwap(ParaLocalStaticLayer* other) {
  using std::swap;
  topic_name_.Swap(&other->topic_name_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(first_map_only_, other->first_map_only_);
  swap(subscribe_to_updates_, other->subscribe_to_updates_);
  swap(track_unknown_space_, other->track_unknown_space_);
  swap(use_maximum_, other->use_maximum_);
  swap(unknown_cost_value_, other->unknown_cost_value_);
  swap(lethal_threshold_, other->lethal_threshold_);
  swap(trinary_map_, other->trinary_map_);
  swap(is_raw_rosmessage_, other->is_raw_rosmessage_);
  swap(is_debug_, other->is_debug_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ParaLocalStaticLayer::GetMetadata() const {
  protobuf_local_5fstatic_5flayer_5fsetting_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_local_5fstatic_5flayer_5fsetting_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace roborts_costmap
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::roborts_costmap::ParaLocalStaticLayer* Arena::CreateMaybeMessage< ::roborts_costmap::ParaLocalStaticLayer >(Arena* arena) {
  return Arena::CreateInternal< ::roborts_costmap::ParaLocalStaticLayer >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
