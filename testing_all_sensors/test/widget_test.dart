// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility in the flutter_test package. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.

import 'package:flutter_test/flutter_test.dart';

import 'package:head_testing_1/main.dart';

void main() {
  testWidgets('Robo Eyes App smoke test', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    await tester.pumpWidget(const RoboEyesApp());

    // Verify that the app title is displayed.
    expect(find.text('🤖 Robo Eyes Controller'), findsOneWidget);
    
    // Verify that the ESP32 status section is present.
    expect(find.text('ESP32 Status:'), findsOneWidget);
    
    // Verify that emotions section is present.
    expect(find.text('🎭 Emotions'), findsOneWidget);
    
    // Verify that responses section is present.
    expect(find.text('✅ Responses'), findsOneWidget);
  });
}
